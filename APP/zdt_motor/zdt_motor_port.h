/**
 ******************************************************************************
 * zdt_motor_port.h — UART 移植层 (STM32F407, 双串口版)
 *
 * ✏️ 需要根据你的硬件改的地方都在这里
 *
 * 本移植层支持两条独立的电机总线：
 *   总线0 ZDT_BUS_CONVEYOR → USART3 → 两条传送带步进电机
 *                                  (默认沿用 103 工程 ID 2、3)
 *   总线1 ZDT_BUS_CHASSIS  → UART4  → 底盘旋转步进电机   (ID 1)
 *
 * 使用步骤：
 *   1. CubeMX 里给 USART3、UART4 配好 DMA (TX+RX 都要) 并生成代码
 *   2. stm32f4xx_it.c 里在 USART3/UART4 的 IRQ 中调用 zdt_port_on_idle()
 *   3. main.c 里在 MX_xxx_Init() 之后调用 zdt_port_init()
 *   4. zdt_init(&电机, 固件, ID, 对应总线的 send 函数)
 ******************************************************************************
 */

#ifndef __ZDT_MOTOR_PORT_H
#define __ZDT_MOTOR_PORT_H

#include "zdt_motor.h"

/* ---- 1. HAL 库头文件 ---- */
#if defined(STM32F1)
  #include "stm32f1xx_hal.h"
#elif defined(STM32F4) || defined(STM32F407xx) || defined(STM32F4XX)
  #include "stm32f4xx_hal.h"
#else
  #include "stm32f4xx_hal.h"
#endif

/* ---- 2. 接收缓冲区大小 (一般不用改) ---- */
#define ZDT_RX_BUF_SIZE    128

/* ---- 3. 总线数量 (两条: 传送带 + 底盘) ---- */
#define ZDT_BUS_MAX        2

/* ---- 总线枚举 ---- */
typedef enum {
    ZDT_BUS_CONVEYOR = 0,   /* USART3 → 两条传送带 */
    ZDT_BUS_CHASSIS  = 1,   /* UART4  → 底盘旋转 */
} zdt_bus_t;

/* ---- 4. 传送带 USART3 直连 TTL UART ----
 * 不再经过 MAX485，PE0 不参与收发方向控制。
 * 保持 USART3、115200、8N1 和原 ZDT 指令帧不变：
 *   PB10 (USART3_TX) -> 两个传送带驱动器 TTL_RX
 *   PB11 (USART3_RX) <- 驱动器 TTL_TX（需要读取应答时接）
 * MCU 和两台驱动器必须共地。RS485 的 A/B 端不能直连 PB10/PB11。
 */

/* ============================================================
 * 🔒 下面的 API 不用动
 * ============================================================ */

void     zdt_port_init(void);                            /* 初始化两条总线(DMA接收+空闲中断) */
void     zdt_port3_send(uint8_t *buf, uint8_t len);      /* USART3 发送(传送带总线) */
void     zdt_port4_send(uint8_t *buf, uint8_t len);      /* UART4 发送(底盘总线) */
void     zdt_port_on_idle(UART_HandleTypeDef *huart);    /* 空闲中断处理(在IRQ里调) */
uint8_t* zdt_port_rx_data(zdt_bus_t bus);                /* 获取某总线接收数据指针 */
uint16_t zdt_port_rx_len(zdt_bus_t bus);                 /* 获取某总线接收数据长度 */
void     zdt_port_rx_clear(zdt_bus_t bus);               /* 清空某总线接收缓冲 */
bool     zdt_port_wait_rx(zdt_bus_t bus, uint32_t ms);   /* 阻塞等待某总线应答(会清缓冲) */

/** 阻塞等一帧应答, 返回帧长(0=超时)。与 zdt_port_wait_rx 的区别: 不清缓冲,
 *  返回后可以用 zdt_port_rx_data(bus) 读内容。用完自己调 zdt_port_rx_clear()。 */
uint16_t zdt_port_wait_frame(zdt_bus_t bus, uint32_t ms);

/* ============================================================
 * 🔍 调试: 把每一帧收发按 HEX 打到串口1
 * ============================================================ */

/** 开/关收发 HEX 打印 (默认开) */
void zdt_dbg_set(bool on);

/** 主循环里每轮调一次: 把中断里存下的应答帧打印出来。
 *  接收打印不能放在中断里做(my_printf 是阻塞的), 所以要靠这个泵。 */
void zdt_dbg_poll(void);

/** 手动打印一段 HEX, tag 是前缀标签 */
void zdt_dbg_hex(const char *tag, const uint8_t *buf, uint16_t len);

/** 扫描总线上的电机: ID 1~16 逐个发读位置命令, 谁应答就打印谁。
 *  返回找到的电机个数。 */
uint8_t zdt_scan_bus(zdt_bus_t bus);

#endif
