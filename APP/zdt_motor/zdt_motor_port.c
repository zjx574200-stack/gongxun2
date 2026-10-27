/**
 ******************************************************************************
 * @file    zdt_motor_port.c
 * @brief   UART移植层实现 (双串口: USART3 传送带 + UART4 底盘)
 *
 * 发送: DMA (每个总线一个发送函数)
 * 接收: DMA + 空闲中断 (每个总线独立缓冲区, 可读回电机状态/位置)
 ******************************************************************************
 */

#include "zdt_motor_port.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;   /* USART3 → 传送带总线 */
extern UART_HandleTypeDef huart4;   /* UART4  → 底盘总线 */

/* 调试打印走串口1。这里直接前向声明, 避免 include mydefine.h 造成头文件循环 */
extern UART_HandleTypeDef huart1;
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

static uint8_t  rx_buf[ZDT_BUS_MAX][ZDT_RX_BUF_SIZE];  /* 各总线接收缓冲 */
static volatile uint16_t rx_len[ZDT_BUS_MAX];           /* 各总线已收长度 */
static UART_HandleTypeDef *bus_uart[ZDT_BUS_MAX];       /* 总线 → UART 句柄 */

/* ---- 调试打印状态 ---- */
#define ZDT_DBG_MAX_BYTES  24          /* 一帧最多打这么多字节, 够看了 */

static bool dbg_on = true;
static const char *bus_name[ZDT_BUS_MAX] = { "U3", "U4" };

/* 接收帧快照: 中断里只做 memcpy + 置标志, 打印留给主循环的 zdt_dbg_poll() */
static uint8_t           dbg_rx_snap[ZDT_BUS_MAX][ZDT_DBG_MAX_BYTES];
static volatile uint16_t dbg_rx_snap_len[ZDT_BUS_MAX];
static volatile bool     dbg_rx_pending[ZDT_BUS_MAX];

static int zdt_bus_of(UART_HandleTypeDef *huart);   /* 前向声明 */

void zdt_port_init(void)
{
    bus_uart[ZDT_BUS_CONVEYOR] = &huart3;
    bus_uart[ZDT_BUS_CHASSIS]  = &huart4;

    for (uint8_t i = 0; i < ZDT_BUS_MAX; i++) {
        rx_len[i] = 0;
        HAL_UART_Receive_DMA(bus_uart[i], rx_buf[i], ZDT_RX_BUF_SIZE);
        __HAL_UART_ENABLE_IT(bus_uart[i], UART_IT_IDLE);
    }
}

/* 底层发送 (DMA, 阻塞等待发完一帧再返回, 防止连发串帧) */
static void zdt_send_uart(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len)
{
    if (!len) return;

    int bus = zdt_bus_of(huart);

    /* 所有上线的字节都从这里过, 在这里打印能拿到真实长度, 不用改 zdt_motor.c */
    if (dbg_on && bus >= 0) {
        char tag[8];
        snprintf(tag, sizeof(tag), "TX%s", bus_name[bus]);
        zdt_dbg_hex(tag, buf, len);
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);                     /* 清掉上次残留的TC */
    HAL_UART_Transmit_DMA(huart, buf, len);

    /* 等最后一个字节发完。加超时: 万一 HAL_UART_Transmit_DMA 因状态忙没真正启动,
     * TC 永远不置位, 裸 while 会死循环把整机卡死 —— 超时后至少能往下走并报出来。 */
    uint32_t t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - t0) > 20) {
            /* 走 dbg_on 开关: USART1 现在是庐山摄像头的线, 不能在关调试时往 PA9 打垃圾 */
            if (dbg_on && bus >= 0) my_printf(&huart1, "!! TX timeout on %s (busy=%u)\r\n",
                                              bus_name[bus], (unsigned)huart->gState);
            break;
        }
    }

}

void zdt_port3_send(uint8_t *buf, uint8_t len)
{
    zdt_send_uart(&huart3, buf, len);
}

void zdt_port4_send(uint8_t *buf, uint8_t len)
{
    zdt_send_uart(&huart4, buf, len);
}

/* 根据 UART 实例找到对应总线 (非电机总线返回 -1) */
static int zdt_bus_of(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) return ZDT_BUS_CONVEYOR;
    if (huart->Instance == UART4)  return ZDT_BUS_CHASSIS;
    return -1;
}

void zdt_port_on_idle(UART_HandleTypeDef *huart)
{
    int bus = zdt_bus_of(huart);
    if (bus < 0) return;

    uint16_t n = ZDT_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    rx_len[bus] = n;

    /* 存一份快照给主循环打印。中断里不能直接 my_printf —— 那是阻塞发送,
     * 一帧四十来个字符在 115200 下要 ~3ms, 卡在中断里会拖垮整个系统。 */
    if (dbg_on && n) {
        uint16_t k = (n > ZDT_DBG_MAX_BYTES) ? ZDT_DBG_MAX_BYTES : n;
        memcpy(dbg_rx_snap[bus], rx_buf[bus], k);
        dbg_rx_snap_len[bus] = k;
        dbg_rx_pending[bus]  = true;
    }

    /* 只中止接收。原来用的 HAL_UART_DMAStop 会把正在进行的 TX DMA 一起掐掉:
     * 连发命令时, 电机对上一条的应答触发 IDLE, 正好把下一条发到一半的帧截断。 */
    HAL_UART_AbortReceive(huart);
    HAL_UART_Receive_DMA(huart, rx_buf[bus], ZDT_RX_BUF_SIZE);
}

uint8_t* zdt_port_rx_data(zdt_bus_t bus)
{
    if (bus >= ZDT_BUS_MAX) return NULL;
    return rx_buf[bus];
}

uint16_t zdt_port_rx_len(zdt_bus_t bus)
{
    if (bus >= ZDT_BUS_MAX) return 0;
    return rx_len[bus];
}

void zdt_port_rx_clear(zdt_bus_t bus)
{
    if (bus < ZDT_BUS_MAX) rx_len[bus] = 0;
}

bool zdt_port_wait_rx(zdt_bus_t bus, uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
        if (zdt_port_rx_len(bus) >= 3) {   /* 至少收到一帧 */
            zdt_port_rx_clear(bus);
            return true;
        }
    }
    return false;
}

uint16_t zdt_port_wait_frame(zdt_bus_t bus, uint32_t ms)
{
    if (bus >= ZDT_BUS_MAX) return 0;

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
        uint16_t n = rx_len[bus];
        if (n >= 3) return n;     /* 不清缓冲, 让调用方能读内容 */
    }
    return 0;
}

/* ============================================================
 *  调试打印
 * ============================================================ */

void zdt_dbg_set(bool on)
{
    dbg_on = on;
}

void zdt_dbg_hex(const char *tag, const uint8_t *buf, uint16_t len)
{
    char line[ZDT_DBG_MAX_BYTES * 3 + 8];
    uint16_t k = (len > ZDT_DBG_MAX_BYTES) ? ZDT_DBG_MAX_BYTES : len;
    uint16_t p = 0;

    for (uint16_t i = 0; i < k; i++) {
        p += snprintf(&line[p], sizeof(line) - p, "%02X ", buf[i]);
    }
    line[p] = '\0';

    my_printf(&huart1, "%s[%u]: %s%s\r\n",
              tag, (unsigned)len, line, (len > k) ? "..." : "");
}

void zdt_dbg_poll(void)
{
    for (uint8_t i = 0; i < ZDT_BUS_MAX; i++) {
        if (!dbg_rx_pending[i]) continue;

        uint16_t n = dbg_rx_snap_len[i];
        dbg_rx_pending[i] = false;          /* 先清标志, 打印期间来的新帧不会丢 */

        char tag[8];
        snprintf(tag, sizeof(tag), "RX%s", bus_name[i]);
        zdt_dbg_hex(tag, dbg_rx_snap[i], n);
    }
}

/* ============================================================
 *  总线扫描: 找出线上有哪些电机 ID
 * ============================================================ */

uint8_t zdt_scan_bus(zdt_bus_t bus)
{
    if (bus >= ZDT_BUS_MAX) return 0;

    bool     saved = dbg_on;
    uint8_t  found = 0;

    dbg_on = false;   /* 扫描时逐条打印太吵, 只报结果 */

    my_printf(&huart1, "--- scan %s, id 1..16 ---\r\n", bus_name[bus]);

    for (uint8_t id = 1; id <= 16; id++) {
        /* 读实时位置: [addr][0x36][0x6B]。在线的电机一定应答, 不在线的没反应 */
        uint8_t frame[3] = { id, ZDT_EMM_RD_POS, 0x6B };

        zdt_port_rx_clear(bus);
        zdt_send_uart(bus_uart[bus], frame, 3);

        uint16_t n = zdt_port_wait_frame(bus, 30);
        if (n) {
            found++;
            my_printf(&huart1, "  found ID=%u  ", (unsigned)id);
            zdt_dbg_hex("reply", zdt_port_rx_data(bus), n);
        }
        zdt_port_rx_clear(bus);
    }

    my_printf(&huart1, "--- scan done, %u motor(s) ---\r\n", (unsigned)found);
    if (!found) {
        my_printf(&huart1, "    no reply: check TX/RX swap, common GND, "
                           "motor power, driver baud (should be 115200)\r\n");
    }

    dbg_on = saved;
    return found;
}
