/**
 * @file    vision_app.c
 * @brief   双 K230 摄像头视觉融合 (F407)
 *
 * 触发: 向两台摄像头各发 "1\n"(0x31 0x0A), 摄像头收到后开始识别。
 * 结果: 帧头 '@', 帧尾回车(0x0D), 中间用英文逗号分隔:
 *   庐山(UART5) = "@形状,数值"
 *   亚博(USART2) = "@形状,颜色,数值"
 *   - 形状(八分类): cone/cube/cuboid/cylinder/hexagonal_prism/
 *     pentagonal_prism/triangular_prism/triangular_pyramid; unknown = 未检测到。
 *   - 颜色(八分类): yellow/blue/green/white/red/purple/orange/gray;
 *     unknown = 未知(只有亚博回)。
 *   - 数值: 识别置信度, 用于两台形状不一致时取高者。
 *
 * 融合: 按源工程十条规则处理两路结果。形状别名采用大小写不敏感的关键词匹配，
 * 兼容摄像头返回 hex/hexagonal、tri/triangular、cuboid/box 等写法。
 *
 * 接线: 庐山=UART5(PD2 RX/PC12 TX), 亚博=USART2(PA3 RX/PA2 TX), 115200 8N1。
 */
#include "vision_app.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
UART_HandleTypeDef huart5;   /* 庐山相机换到 UART5(PC12 TX / PD2 RX), 手动初始化, 见 vision_init() */

volatile int32_t  g_vision_shape     = -1;
volatile int32_t  g_vision_color     = -1;
volatile uint8_t  g_vision_new_frame = 0;
volatile uint32_t g_vision_rx_bytes  = 0;
volatile uint32_t g_vision_sent_ls   = 0;   /* 本轮累计发给庐山(UART5)的 "1\n" 次数, 每轮开始清零 */
volatile float     g_vision_score    = 0.0f;
volatile const char *g_vision_reason = "NO TARGET";
volatile int32_t   g_vision_shape0   = -1;  /* 庐山原始形状 */
volatile int32_t   g_vision_shape1   = -1;  /* 亚博原始形状 */
volatile int32_t   g_vision_color1   = -1;  /* 亚博原始颜色 */

/* ---------- 串口环形缓冲 (两路: port0=UART5=庐山, port1=USART2=亚博) ---------- */
#define VISION_RX_SIZE   512

static volatile uint8_t  s_rx[2][VISION_RX_SIZE];
static volatile uint16_t s_head[2];
static volatile uint16_t s_tail[2];
static volatile uint8_t  s_fault[2];
static volatile uint32_t s_rx_count[2];   /* 本轮累计收到的字节数(含坏字节) */
static volatile uint8_t  s_rx_error[2];   /* 本轮是否出现帧错误(ORE/NE/FE/PE) */

static uint8_t vision_read_byte(uint8_t port, uint8_t *byte)
{
    if (port >= 2 || s_tail[port] == s_head[port]) {
        return 0;
    }
    *byte = s_rx[port][s_tail[port]];
    s_tail[port] = (uint16_t)((s_tail[port] + 1) & (VISION_RX_SIZE - 1));
    return 1;
}

/** RXNE 接收中断处理: 读 SR/DR 清 RXNE, 把字节压进对应串口环形缓冲。
 *  在 stm32f4xx_it.c 的 UART5/USART2 IRQ 里调用(HAL_UART_IRQHandler 之前)。 */
void vision_uart_irq(UART_HandleTypeDef *huart)
{
    uint8_t  port;
    uint32_t status;
    uint16_t next;
    uint8_t  byte;

    if (huart->Instance == UART5) {
        port = 0;
    } else if (huart->Instance == USART2) {
        port = 1;
    } else {
        return;
    }

    status = huart->Instance->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        byte = (uint8_t)(huart->Instance->DR & 0xFF);
        s_rx_count[port]++;                          /* 任何字节(含坏字节)都计数 */
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            s_fault[port] = 1;      /* 线路错误, 这路作废, 待清空 */
            s_rx_error[port] = 1;   /* 帧错误: 波特率不对/接反/共地问题 */
            return;
        }
        next = (uint16_t)((s_head[port] + 1) & (VISION_RX_SIZE - 1));
        if (next == s_tail[port]) {
            s_fault[port] = 1;      /* 缓冲满, 溢出 */
            return;
        }
        s_rx[port][s_head[port]] = byte;
        s_head[port] = next;
        g_vision_rx_bytes++;
    }
}

static void vision_send(uint8_t port, const uint8_t *data, uint16_t n)
{
    UART_HandleTypeDef *huart = (port == 0) ? &huart5 : &huart2;
    if (port == 0) {
        g_vision_sent_ls++;   /* 每给庐山发一次 "1\n" 就 +1, OLED 上能看到确实在发 */
    }
    HAL_UART_Transmit(huart, (uint8_t *)data, n, 100);
}

/* ---------- "@形状,颜色\r" 行解析 ---------- */

typedef struct {
    int32_t shape;   /* 0~7, -1 = 未知/没收到 */
    int32_t color;   /* 0~7, -1 = 未知/没收到(只有亚博回颜色) */
    float   score;   /* 识别置信度数值, 用于形状不一致时取高者 */
    uint8_t got;     /* 本轮是否已收到有效回包 */
} CamResult;

static CamResult s_cam[2];
static char      s_line[2][48];
static uint8_t   s_line_len[2];

/* 关键字子串匹配(不区分大小写)。摄像头固件版本不同，有的返回完整英文名，
 * 有的只返回 hex/tri/penta/cyl 等缩写；这里统一归一化到同一组形状编号。
 * 表项按“长词在前”排列，避免 cuboid 先命中 cube、triangular_pyramid 先命中 tri。 */
static uint8_t keyword_present(const char *s, uint8_t n, const char *keyword)
{
    uint8_t klen = (uint8_t)strlen(keyword);
    uint8_t i;
    uint8_t j;
    char    a;
    char    b;

    if (klen == 0 || klen > n) {
        return 0;
    }
    for (i = 0; (uint16_t)i + klen <= n; i++) {
        for (j = 0; j < klen; j++) {
            a = s[i + j];
            b = keyword[j];
            if (a >= 'A' && a <= 'Z') {
                a = (char)(a + ('a' - 'A'));
            }
            if (b >= 'A' && b <= 'Z') {
                b = (char)(b + ('a' - 'A'));
            }
            if (a != b) {
                break;
            }
        }
        if (j == klen) {
            return 1;
        }
    }
    return 0;
}

static int32_t shape_id(const char *s, uint8_t n)
{
    typedef struct { int32_t id; const char *keyword; } ShapeKeyword;
    static const ShapeKeyword tab[] = {
        /* Full names first. */
        {7, "triangular_pyramid"}, {6, "triangular_prism"},
        {4, "hexagonal_prism"},   {5, "pentagonal_prism"},
        {3, "cylinder"},          {2, "cuboid"},
        {1, "cube"},              {0, "cone"},
        /* Aliases used by the different K230 camera scripts. */
        {4, "hexagonal"}, {4, "hexagon"}, {4, "hex"},
        {5, "pentagonal"}, {5, "pentagon"}, {5, "penta"},
        {7, "pyramid"},
        {6, "triangular"}, {6, "triangle"}, {6, "tri"},
        {3, "cyl"}, {3, "round"}, {3, "circular"},
        {2, "rectangular"}, {2, "rectangle"}, {2, "rect"}, {2, "box"},
        {1, "square"},
        {0, "conical"},
        {-1, 0}
    };
    uint8_t i;

    for (i = 0; tab[i].keyword != 0; i++) {
        if (keyword_present(s, n, tab[i].keyword)) {
            return tab[i].id;
        }
    }
    return -1;
}

static int32_t color_id(const char *s, uint8_t n)
{
    typedef struct { int32_t id; const char *keyword; } ColorKeyword;
    static const ColorKeyword tab[] = {
        {0, "yellow"}, {1, "blue"}, {2, "green"}, {3, "white"}, {4, "red"},
        {5, "purple"}, {5, "violet"}, {6, "orange"},
        {7, "gray"}, {7, "grey"},
        {-1, 0}
    };
    uint8_t i;

    for (i = 0; tab[i].keyword != 0; i++) {
        if (keyword_present(s, n, tab[i].keyword)) {
            return tab[i].id;
        }
    }
    return -1;
}

/** 解析数值(置信度): 支持整数("95")和小数("0.98"), 解析失败返回 0。 */
static float parse_score(const char *s, uint8_t n)
{
    float   v = 0.0f;
    float   frac = 0.1f;
    uint8_t i = 0;
    uint8_t seen_dot = 0;
    uint8_t seen_digit = 0;

    while (i < n && (s[i] == ' ' || s[i] == '\t')) {
        i++;
    }
    for (; i < n; i++) {
        char c = s[i];
        if (c >= '0' && c <= '9') {
            seen_digit = 1;
            if (seen_dot) {
                v += (float)(c - '0') * frac;
                frac *= 0.1f;
            } else {
                v = v * 10.0f + (float)(c - '0');
            }
        } else if (c == '.' && !seen_dot) {
            seen_dot = 1;
        } else {
            break;
        }
    }
    return seen_digit ? v : 0.0f;
}

/** 处理一条完整行(大小写按名字表严格匹配):
 *  庐山(port0): "@形状,数值";  亚博(port1): "@形状,颜色,数值"。
 *  名字对不上(含 unknown) → 对应字段 -1, 数值解析失败 → 0, 但仍算回包(got=1)。 */
static void process_line(uint8_t port, const char *line)
{
    const char *at = strchr(line, '@');
    const char *c1, *c2;
    int32_t shape = -1, color = -1;
    float   score = 0.0f;

    if (!at) {
        return;
    }
    at++;
    c1 = strchr(at, ',');
    if (!c1) {
        return;
    }

    shape = shape_id(at, (uint8_t)(c1 - at));

    if (port == 0) {
        /* 庐山: "@形状,数值" */
        score = parse_score(c1 + 1, (uint8_t)strlen(c1 + 1));
    } else {
        /* 亚博: "@形状,颜色,数值" */
        c2 = strchr(c1 + 1, ',');
        if (!c2) {
            return;
        }
        color = color_id(c1 + 1, (uint8_t)(c2 - (c1 + 1)));
        score = parse_score(c2 + 1, (uint8_t)strlen(c2 + 1));
    }

    s_cam[port].shape = shape;
    s_cam[port].color = color;
    s_cam[port].score = score;
    s_cam[port].got   = 1;
}

/** 喂一个字节: 遇 \r 或 \n 判为行尾, 交给 process_line。 */
static void line_feed(uint8_t port, uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (s_line_len[port] > 0) {
            s_line[port][s_line_len[port]] = '\0';
            process_line(port, s_line[port]);
        }
        s_line_len[port] = 0;
    } else if (s_line_len[port] < sizeof(s_line[port]) - 1) {
        s_line[port][s_line_len[port]++] = (char)byte;
    } else {
        s_line_len[port] = 0;   /* 行太长, 丢弃 */
    }
}

/** 取走并清掉某路的线路错误标志(临界区, 防止和 IRQ 竞争)。 */
static uint8_t vision_take_fault(uint8_t port)
{
    uint32_t primask;
    uint8_t  f;

    if (port >= 2) {
        return 1;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    f = s_fault[port];
    s_fault[port] = 0;
    if (f) {
        s_tail[port] = s_head[port];   /* 出错这路: 清空缓冲 + 清行缓冲 */
        s_line_len[port] = 0;
    }
    if (!primask) {
        __enable_irq();
    }
    return f;
}

/** 清空两路环形缓冲(丢弃触发前的陈旧数据)。 */
static void vision_flush(void)
{
    uint8_t v;
    for (v = 0; v < 2; v++) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_tail[v] = s_head[v];
        s_fault[v] = 0;
        s_rx_count[v] = 0;    /* 新一轮开始, 清零字节计数和帧错误 */
        s_rx_error[v] = 0;
        if (!primask) {
            __enable_irq();
        }
        s_line_len[v] = 0;
    }
}

/** 把两路环形缓冲里的字节全部出队, 喂进行解析器。主循环里反复调用。 */
static void vision_pump(void)
{
    uint8_t v, byte;
    for (v = 0; v < 2; v++) {
        vision_take_fault(v);
        while (vision_read_byte(v, &byte)) {
            line_feed(v, byte);
        }
    }
}

/* ---------- 融合 ---------- */

/* 形状编号(与 shape_id() 和 OLED/转盘表一致)。 */
#define SHAPE_CONE        0
#define SHAPE_CUBE        1
#define SHAPE_CUBOID      2
#define SHAPE_CYLINDER    3
#define SHAPE_HEX_PRISM   4
#define SHAPE_PENT_PRISM  5
#define SHAPE_TRI_PRISM   6
#define SHAPE_TRI_PYRAMID 7

/* unknown 回包或置信度解析失败都视为无效，不能让 0 分结果参与冲突裁决。 */
static uint8_t cam_valid(int32_t shape, float score)
{
    return (uint8_t)(shape >= 0 && shape <= 7 && score > 0.0f);
}

/* 源工程的十条融合规则，顺序有意义，命中即停。
 * a=庐山(port0)，b=亚博(port1)。颜色始终取亚博结果。 */
static void fuse_and_publish(void)
{
    int32_t s0 = s_cam[0].shape;
    int32_t s1 = s_cam[1].shape;
    float   c0 = s_cam[0].score;
    float   c1 = s_cam[1].score;
    uint8_t v0 = cam_valid(s0, c0);
    uint8_t v1 = cam_valid(s1, c1);
    int32_t shape;
    float   fused;
    const char *reason;

    /* 保留两路原始结果，供 OLED 判断“哪一路识别错/没回”。 */
    g_vision_shape0 = s0;
    g_vision_shape1 = s1;
    g_vision_color1 = s_cam[1].color;

    if (v0 != v1) {
        /* 1. 一路有效、一路无效: 采用有效一路。 */
        shape = v0 ? s0 : s1;
        fused = v0 ? c0 : c1;
        reason = "SINGLE VALID";
    } else if (!v0) {
        /* 2. 两路都无效/unknown。 */
        shape = -1;
        fused = 0.0f;
        reason = "NO TARGET";
    } else if (s0 == s1) {
        /* 3. 两路同类，分数取较低值。 */
        shape = s0;
        fused = (c0 <= c1) ? c0 : c1;
        reason = (s0 == SHAPE_CUBE) ? "BOTH CUBE" : "BOTH AGREE";
    } else if ((s0 == SHAPE_CUBE && s1 == SHAPE_CUBOID) ||
               (s0 == SHAPE_CUBOID && s1 == SHAPE_CUBE)) {
        /* 4. cube 与 cuboid 冲突时固定选 cube。 */
        shape = SHAPE_CUBE;
        fused = (s0 == SHAPE_CUBE) ? c0 : c1;
        reason = "CUBE VS CUBOID";
    } else if (s0 == SHAPE_CUBOID || s1 == SHAPE_CUBOID) {
        /* 5. cuboid 与其它形状冲突时淘汰 cuboid。 */
        shape = (s0 == SHAPE_CUBOID) ? s1 : s0;
        fused = (s0 == SHAPE_CUBOID) ? c1 : c0;
        reason = "NOT CUBOID";
    } else if (s0 == SHAPE_CUBE || s1 == SHAPE_CUBE) {
        /* 6. cube 与非 cuboid 冲突时淘汰 cube。 */
        shape = (s0 == SHAPE_CUBE) ? s1 : s0;
        fused = (s0 == SHAPE_CUBE) ? c1 : c0;
        reason = "NOT CUBE";
    } else if (s0 == SHAPE_PENT_PRISM && s1 == SHAPE_HEX_PRISM) {
        /* 7. 庐山五棱柱 + 亚博六棱柱 -> 五棱柱。 */
        shape = SHAPE_PENT_PRISM;
        fused = c0;
        reason = "PENT VS HEX";
    } else if (s0 == SHAPE_CONE && s1 == SHAPE_TRI_PRISM) {
        /* 8. 庐山圆锥 + 亚博三棱柱 -> 圆锥。 */
        shape = SHAPE_CONE;
        fused = c0;
        reason = "CONE VS TRI";
    } else if (c0 > c1) {
        /* 9. 其它冲突，取置信度高者。 */
        shape = s0;
        fused = c0;
        reason = "HIGH CONF";
    } else if (c1 > c0) {
        shape = s1;
        fused = c1;
        reason = "HIGH CONF";
    } else {
        /* 10. 其它冲突且分数相同，无法裁决。 */
        shape = -1;
        fused = 0.0f;
        reason = "SAME SCORE";
    }

    g_vision_shape = shape;
    g_vision_color = s_cam[1].color;
    g_vision_score = fused;
    g_vision_reason = reason;
    g_vision_new_frame = 1;
}

/* ---------- 一轮识别 ---------- */

int32_t vision_capture(uint32_t timeout_ms)
{
    uint32_t t0, now;
    uint8_t  second_sent = 0;

    /* 丢触发前的陈旧数据, 避免上一轮迟到的回包被当成这轮结果 */
    vision_flush();
    s_cam[0].shape = s_cam[0].color = -1; s_cam[0].got = 0;
    s_cam[1].shape = s_cam[1].color = -1; s_cam[1].got = 0;
    g_vision_sent_ls = 0;   /* 本轮发送计数清零 */

    /* 第一帧立即触发；500 ms 后补发一次，兼容摄像头刚启动/忙于上一帧的情况。 */
    vision_send(0, (const uint8_t *)"1\n", 2);
    vision_send(1, (const uint8_t *)"1\n", 2);

    t0 = HAL_GetTick();

    while (!(s_cam[0].got && s_cam[1].got)) {
        now = HAL_GetTick();

        if (!second_sent && (uint32_t)(now - t0) >= 500U) {
            if (!s_cam[0].got) {
                vision_send(0, (const uint8_t *)"1\n", 2);
            }
            if (!s_cam[1].got) {
                vision_send(1, (const uint8_t *)"1\n", 2);
            }
            second_sent = 1;
        }

        vision_pump();

        if ((uint32_t)(now - t0) >= timeout_ms) {
            break;
        }
        HAL_Delay(1);
    }

    fuse_and_publish();
    return (s_cam[0].got && s_cam[1].got) ? 0 : -1;
}

void vision_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    s_cam[0].shape = s_cam[0].color = -1; s_cam[0].got = 0;
    s_cam[1].shape = s_cam[1].color = -1; s_cam[1].got = 0;
    s_line_len[0] = s_line_len[1] = 0;

    /* 庐山相机换到 UART5(PC12=TX / PD2=RX), 手动初始化, 不经过 CubeMX
     * (避免 CubeMX 重新生成时动到 USART1/2/3/UART4 的引脚)。UART5 挂 APB1, AF8。 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_12;                 /* PC12 = UART5_TX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin       = GPIO_PIN_2;                  /* PD2 = UART5_RX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;                 /* 悬空时保持稳定 */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);

    huart5.Instance          = UART5;
    huart5.Init.BaudRate     = 115200;
    huart5.Init.WordLength   = UART_WORDLENGTH_8B;
    huart5.Init.StopBits     = UART_STOPBITS_1;
    huart5.Init.Parity       = UART_PARITY_NONE;
    huart5.Init.Mode         = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart5);

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

/* ---------- 动态格位分配 ---------- */

#define VISION_SLOT_MAX   6

static int32_t s_slot_shape[VISION_SLOT_MAX];
static int32_t s_slot_color[VISION_SLOT_MAX];
static int32_t s_slot_count = 0;

/** 形状+颜色 → 动态格位序号(1~6)。首次遇到的组合占下一个空格; 已见过返回原序号;
 * 形状/颜色非法返回 0; 6 格用满后新类别沿用 1 号格。 */
int32_t vision_slot_index(int32_t shape, int32_t color)
{
    int32_t i;

    if (shape < 0 || shape > 7 || color < 0 || color > 7) {
        return 0;
    }

    for (i = 0; i < s_slot_count; i++) {
        if (s_slot_shape[i] == shape && s_slot_color[i] == color) {
            return i + 1;
        }
    }

    if (s_slot_count < VISION_SLOT_MAX) {
        s_slot_shape[s_slot_count] = shape;
        s_slot_color[s_slot_count] = color;
        s_slot_count++;
        return s_slot_count;
    }

    return 1;   /* 6 格都用满, 新类别统一归到 1 号格 */
}

/** 形状+颜色 → 转盘格角度。两者完全一致才同格, 动态分配; 非法返回 0°(不分)。 */
float vision_slot_angle(int32_t shape, int32_t color)
{
    int32_t idx = vision_slot_index(shape, color);

    if (idx <= 0) {
        return 0.0f;
    }
    return (float)idx * VISION_SLOT_ANGLE_DEG + VISION_ANGLE_OFFSET_DEG;
}

/* ---------- 调试命名 ---------- */

const char *vision_shape_name(int32_t shape)
{
    static const char *const names[8] = {
        "cone", "cube", "cuboid", "cylinder",
        "hexagonal_prism", "pentagonal_prism",
        "triangular_prism", "triangular_pyramid"
    };
    if (shape < 0 || shape > 7) {
        return "unknown";
    }
    return names[shape];
}

const char *vision_color_name(int32_t color)
{
    static const char *const names[8] = {
        "yellow", "blue", "green", "white", "red", "purple", "orange", "gray"
    };
    if (color < 0 || color > 7) {
        return "unknown";
    }
    return names[color];
}

const char *vision_shape_short(int32_t shape)
{
    static const char *const names[8] = {
        "cone", "cube", "cuboid", "cylin",
        "hexpri", "penpri", "tripri", "tripyr"
    };
    if (shape < 0 || shape > 7) {
        return "unknown";
    }
    return names[shape];
}

void vision_get_cam(uint8_t port, VisionCamInfo *out)
{
    if (!out) {
        return;
    }
    if (port >= 2) {
        out->shape = out->color = -1;
        out->score = 0.0f;
        out->got   = 0;
        out->rx_bytes = 0;
        out->rx_error = 0;
        return;
    }
    out->shape    = s_cam[port].shape;
    out->color    = s_cam[port].color;
    out->score    = s_cam[port].score;
    out->got      = s_cam[port].got;
    out->rx_bytes = s_rx_count[port];
    out->rx_error = s_rx_error[port];
}

/* 读取最近一轮两路接收诊断，便于区分“没有字节进来”和“字节进来但帧/关键词不对”。 */
void vision_debug_get(int32_t out[6])
{
    if (!out) {
        return;
    }
    out[0] = (int32_t)s_rx_count[0];
    out[1] = (int32_t)s_rx_error[0];
    out[2] = (int32_t)s_cam[0].got;
    out[3] = (int32_t)s_rx_count[1];
    out[4] = (int32_t)s_rx_error[1];
    out[5] = (int32_t)s_cam[1].got;
}
