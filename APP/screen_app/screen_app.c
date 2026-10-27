/**
 * @file    screen_app.c
 * @brief   淘晶驰(TJC)串口屏模块 (F407) —— 实现
 *
 * 手动初始化 USART6(PC6=TX / PC7=RX, 115200), 不经过 CubeMX, 防止 CubeMX
 * 重新生成时把 USART1/2/3/UART4 的引脚配置覆盖掉(和 F103 版 vision_init 同一思路)。
 * 屏只用来显示, 本模块只发不收, 不开 RX 中断。
 */
#include "screen_app.h"
#include "vision_app/vision_app.h"   /* vision_shape_name / vision_color_name / vision_slot_index */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* TJC 协议结束符: 每条指令后固定追加 0xFF 0xFF 0xFF */
static const uint8_t tjc_end[3] = {0xFF, 0xFF, 0xFF};

static UART_HandleTypeDef huart_screen;

/* 图片库列号(见 图片库ID对照表.md), 顺序: 0=六棱柱 1=三棱柱 2=四棱锥 3=五棱柱
 *   4=圆柱 5=长方体 6=正方体 7=圆锥。图片只由"形状列 + 颜色"决定,
 *   和屏上显示在哪一行无关。 */
static const int8_t shape_to_col[8] = {
     7,  /* 0 cone 圆锥 */
     6,  /* 1 cube 正方体 */
     5,  /* 2 cuboid 长方体 */
     4,  /* 3 cylinder 圆柱 */
     0,  /* 4 hexagonal_prism 六棱柱 */
     3,  /* 5 pentagonal_prism 五棱柱 */
     1,  /* 6 triangular_prism 三棱柱 */
     2,  /* 7 triangular_pyramid 三棱锥 —— 图片库是四棱锥, 视觉最接近 */
};

/* 屏上颜色偏移: 白0 红1 黄2 蓝3 绿4 紫5 橙6 灰7。
 * F407 颜色编号(yellow0 blue1 green2 white3 red4 purple5 orange6 gray7)
 * → 屏偏移。 */
static const uint8_t color_to_off[8] = {
    2,  /* 0 yellow → 黄 */
    3,  /* 1 blue   → 蓝 */
    4,  /* 2 green  → 绿 */
    0,  /* 3 white  → 白 */
    1,  /* 4 red    → 红 */
    5,  /* 5 purple → 紫 */
    6,  /* 6 orange → 橙 */
    7,  /* 7 gray   → 灰 */
};

static uint16_t s_count[7];   /* 每个序号(动态格位)分拣成功累计计数 */
static uint16_t s_total;      /* 总计数(所有序号累计), 显示在最后一行(行6) */

void screen_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t row;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    /* PC6 = USART6_TX, PC7 = USART6_RX, 复用推挽, AF8 */
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* USART6 参数(115200, 与 test_v2.HMI 屏工程一致) */
    huart_screen.Instance          = USART6;
    huart_screen.Init.BaudRate     = 115200;
    huart_screen.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_screen.Init.StopBits     = UART_STOPBITS_1;
    huart_screen.Init.Parity       = UART_PARITY_NONE;
    huart_screen.Init.Mode         = UART_MODE_TX_RX;
    huart_screen.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_screen.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_screen);

    memset(s_count, 0, sizeof(s_count));
    s_total = 0;

    /* 清掉屏幕可能保留的上一轮内容，保证第一件货物从第一行开始。 */
    for (row = 0; row < 6; row++) {
        screen_set_text(row, "");
        screen_set_num(row, 0);
        screen_send("vis p%u,0", (unsigned)row);
    }

    /* 最后一行(行6)固定作"总计"行: 隐藏该行图片控件, 显示 Total + 0 */
    screen_set_text(6, "Total");
    screen_set_num(6, 0);
    screen_send("vis p6,0");
}

void screen_send(const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0 || (size_t)len >= sizeof(buf)) {
        return;   /* 空串或截断, 不发 */
    }

    HAL_UART_Transmit(&huart_screen, (uint8_t *)buf, (uint16_t)len, 100);
    HAL_UART_Transmit(&huart_screen, (uint8_t *)tjc_end, 3, 100);
}

void screen_set_pic(uint8_t row, uint8_t pic_id)
{
    screen_send("p%u.pic=%u", (unsigned)row, (unsigned)pic_id);
}

void screen_set_num(uint8_t row, int32_t val)
{
    screen_send("n%u.val=%ld", (unsigned)row, (long)val);
}

void screen_set_text(uint8_t row, const char *s)
{
    if (s == 0) {
        return;
    }
    screen_send("t%u.txt=\"%s\"", (unsigned)row, s);
}

uint8_t screen_show_goods(int32_t shape, int32_t color)
{
    int32_t slot;
    uint8_t row, off, pic_id;
    int8_t  col;               /* 有符号: 圆锥 = -1, 用来跳过图片 */
    char    name[40];

    if (shape < 0 || shape > 7 || color < 0 || color > 7) {
        return 0;
    }

    /* 动态序号: 首次遇到的(形状,颜色)组合占下一个序号, 和转盘格位共用同一张表 */
    slot = vision_slot_index(shape, color);
    if (slot <= 0) {
        return 0;   /* 6 格都用满 */
    }
    row = (uint8_t)(slot - 1);

    /* 货名: "形状-颜色"(英文, 避开 GBK/UTF-8 中文源码编译坑) */
    snprintf(name, sizeof(name), "%s-%s",
             vision_shape_name(shape), vision_color_name(color));

    /* 图片: 形状列 + 颜色偏移 */
    col = shape_to_col[shape];
    if (col >= 0) {
        off    = color_to_off[color];
        pic_id = (uint8_t)(3 + col * 8 + off);
        screen_set_pic(row, pic_id);
        screen_send("vis p%u,1", (unsigned)row);
    }

    s_count[row]++;   /* 该序号累计 +1 */
    screen_set_num(row, (int32_t)s_count[row]);
    screen_set_text(row, name);

    /* 最后一行(行6)更新总计 */
    s_total++;
    screen_set_num(6, (int32_t)s_total);
    return 1;
}
