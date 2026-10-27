/**
 * @file    oled_app.c
 * @brief   SSD1306 128x64 OLED 驱动 (软件 I2C, PB8=SCL/PB9=SDA, 地址 0x3C)
 *
 * 移植自 F103 Solid8 工程的 Hardware/OLED.c, 改动只有:
 *   - GPIO 读写从 F103 标准库(GPIO_WriteBit) 换成 HAL(HAL_GPIO_WritePin);
 *   - 软件 I2C 使用较慢的可靠时序，避免 168MHz 下超出 OLED 的 I2C 速率;
 *   - 上电延时改用 HAL_Delay(100)。
 * 时序逻辑(Start/Stop/SendByte/命令帧/初始化序列)与 F103 版完全一致。
 */
#include "oled_app.h"
#include "OLED_Font.h"
#include "vision_app/vision_app.h"

#include <stdio.h>

static const char *oled_shape_short(int32_t shape);
static const char *oled_color_short(int32_t color);

/* ---- 引脚 ---- */
#define OLED_SCL_PORT   GPIOB
#define OLED_SCL_PIN    GPIO_PIN_8
#define OLED_SDA_PORT   GPIOB
#define OLED_SDA_PIN    GPIO_PIN_9

/* 常见 OLED 模块有 0x3C 或 0x3D 两种地址。两种地址都发送，
 * 不影响单屏使用，可避免模块地址焊盘配置不同导致整屏无显示。 */
#define OLED_ADDR_3C    0x78U
#define OLED_ADDR_3D    0x7AU

#define OLED_W_SCL(x)   HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_W_SDA(x)   HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* ---- 软件 I2C 节拍延时: 168MHz 下约 2~3us 半周期，约 150~200kHz ---- */
static void OLED_I2C_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 120; i++) {
        __NOP();
    }
}

static void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏, 和 F103 版一致 */
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

static void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_I2C_Delay();
    OLED_W_SDA(0);
    OLED_I2C_Delay();
    OLED_W_SCL(0);
}

static void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_I2C_Delay();
    OLED_W_SDA(1);
    OLED_I2C_Delay();
}

static void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        OLED_W_SDA((Byte & (0x80 >> i)) ? 1 : 0);
        OLED_W_SCL(1);
        OLED_I2C_Delay();
        OLED_W_SCL(0);
        OLED_I2C_Delay();
    }
    /* ACK 位：释放 SDA，再给从机一个完整的应答时钟。
     * 不读取 ACK，保持与原工程的盲写方式一致。 */
    OLED_W_SDA(1);
    OLED_I2C_Delay();
    OLED_W_SCL(1);
    OLED_I2C_Delay();
    OLED_W_SCL(0);
    OLED_I2C_Delay();
}

static void OLED_WriteCommandTo(uint8_t Address, uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(Address);
    OLED_I2C_SendByte(0x00);   /* 命令字节 */
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

static void OLED_WriteCommand(uint8_t Command)
{
    OLED_WriteCommandTo(OLED_ADDR_3C, Command);
    OLED_WriteCommandTo(OLED_ADDR_3D, Command);
}

static void OLED_WriteDataTo(uint8_t Address, uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(Address);
    OLED_I2C_SendByte(0x40);   /* 数据字节 */
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t Data)
{
    OLED_WriteDataTo(OLED_ADDR_3C, Data);
    OLED_WriteDataTo(OLED_ADDR_3D, Data);
}

static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                  /* 页地址 */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));  /* 列地址高 4 位 */
    OLED_WriteCommand(0x00 | (X & 0x0F));         /* 列地址低 4 位 */
}

void oled_clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

static void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    if ((unsigned char)Char < ' ' || (unsigned char)Char > '~') {
        Char = ' ';
    }
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);      /* 上半部分 */
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);  /* 下半部分 */
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

void oled_show_string(uint8_t Line, uint8_t Column, const char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void oled_show_num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, (char)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

void oled_clear_line(uint8_t Line)
{
    uint8_t i;
    for (i = 0; i < 16; i++) {
        OLED_ShowChar(Line, 1 + i, ' ');
    }
}

void oled_show_status(uint8_t sent)
{
    /* 只刷第 1 行状态, 不清屏 —— 第 2~4 行保留上一轮两台摄像头的识别结果 */
    oled_clear_line(1);
    if (sent) {
        oled_show_string(1, 1, "SEND OK");     /* 发送成功 */
    } else {
        oled_show_string(1, 1, "NOT SEND");    /* 未发送成功 */
    }
}

void oled_show_stage(const char *stage)
{
    oled_clear_line(1);
    if (stage != NULL) {
        oled_show_string(1, 1, stage);
    }
}

void oled_show_camera_diagnostics(const int32_t diag[6])
{
    char line[17];

    if (diag == NULL) {
        return;
    }

    oled_clear();

    snprintf(line, sizeof(line), "LS:%s", vision_shape_short(g_vision_shape0));
    oled_show_string(1, 1, line);
    snprintf(line, sizeof(line), "YB:%s %s", vision_shape_short(g_vision_shape1),
             oled_color_short(g_vision_color1));
    oled_show_string(2, 1, line);
    snprintf(line, sizeof(line), "F:%s %s", vision_shape_short(g_vision_shape),
             oled_color_short(g_vision_color));
    oled_show_string(3, 1, line);
    snprintf(line, sizeof(line), "L%ld Y%ld E%ld/%ld",
             (long)diag[0], (long)diag[3], (long)diag[1], (long)diag[4]);
    oled_show_string(4, 1, line);
}

/* OLED 只有 16 个 8x16 字符格。把多个字段拼成一行并截断，避免
 * 长形状名或融合规则覆盖到下一行。 */
static void oled_show_composed_line(uint8_t line,
                                    const char *p0,
                                    const char *p1,
                                    const char *p2,
                                    const char *p3)
{
    const char *parts[4];
    char        text[17];
    uint8_t     part;
    uint8_t     n;
    uint8_t     i;

    parts[0] = p0;
    parts[1] = p1;
    parts[2] = p2;
    parts[3] = p3;
    n = 0;

    for (part = 0; part < 4; part++) {
        if (parts[part] == NULL) {
            continue;
        }
        for (i = 0; parts[part][i] != '\0' && n < 16; i++) {
            text[n++] = parts[part][i];
        }
    }
    text[n] = '\0';

    oled_clear_line(line);
    oled_show_string(line, 1, text);
}

/* Short names keep the complete camera/fusion view inside one 16-character row. */
static const char *oled_shape_short(int32_t shape)
{
    switch (shape) {
        case 0: return "cone";
        case 1: return "cube";
        case 2: return "cuboid";
        case 3: return "cyl";
        case 4: return "hex";
        case 5: return "penta";
        case 6: return "tri";
        case 7: return "tripyr";
        default: return "unknown";
    }
}

static const char *oled_color_short(int32_t color)
{
    switch (color) {
        case 0: return "yellow";
        case 1: return "blue";
        case 2: return "green";
        case 3: return "white";
        case 4: return "red";
        case 5: return "purple";
        case 6: return "orange";
        case 7: return "gray";
        default: return "unknown";
    }
}

void oled_show_cameras(int32_t shape0, int32_t shape1, int32_t color1,
                       int32_t fused_shape, int32_t fused_color,
                       const char *reason)
{
    /* Keep the labels stable: they are useful when the display is viewed
     * without a debugger and match the source-board OLED terminology. */
    oled_show_composed_line(1, "LS:", oled_shape_short(shape0), NULL, NULL);
    oled_show_composed_line(2, "YB:", oled_shape_short(shape1), " ",
                            oled_color_short(color1));
    oled_show_composed_line(3, "F:", oled_shape_short(fused_shape), " ",
                            oled_color_short(fused_color));
    oled_show_composed_line(4, "R:", reason ? reason : "unknown", NULL, NULL);
}

void oled_init(void)
{
    HAL_Delay(100);   /* 上电延时(替代 F103 版的忙等双重循环) */

    OLED_I2C_Init();

    OLED_WriteCommand(0xAE);   /* 关闭显示 */

    OLED_WriteCommand(0xD5);   /* 显示时钟分频比/振荡器频率 */
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);   /* 多路复用率 */
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);   /* 显示偏移 */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);   /* 显示起始行 */

    OLED_WriteCommand(0xA1);   /* 段重映射(左右方向) */

    OLED_WriteCommand(0xC8);   /* COM 扫描方向(上下方向) */

    OLED_WriteCommand(0xDA);   /* COM 引脚硬件配置 */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);   /* 对比度 */
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);   /* 预充电周期 */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);   /* VCOMH 取消选择级别 */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);   /* 整个显示打开 */

    OLED_WriteCommand(0xA6);   /* 正常显示(不倒转) */

    OLED_WriteCommand(0x8D);   /* 充电泵 */
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);   /* 开启显示 */

    oled_clear();
}
