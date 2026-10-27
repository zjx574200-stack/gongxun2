#ifndef __OLED_APP_H
#define __OLED_APP_H

/**
 * @file    oled_app.h
 * @brief   SSD1306 128x64 OLED 状态屏 (软件 I2C, 移植自 F103 Solid8 工程)
 *
 * 引脚: SCL = PB8, SDA = PB9 (开漏 + 上拉), 兼容从机地址 0x3C/0x3D。
 *       电源另接 3.3V + GND, 所以 OLED 模块一共 4 根线(VCC/GND/SCL/SDA)。
 *
 * 用途: 分拣状态指示 —— 触发摄像头发 CAPTURE 之后显示 "SEND OK"(发送成功),
 *       直到摄像头回传数据; 其余时间显示 "NOT SEND"(未发送成功)。
 *
 * 注意: 字模库 OLED_Font.h 是纯 ASCII(8x16), 不含中文字模,
 *       所以 "发送成功/未发送成功" 用英文 "SEND OK"/"NOT SEND" 显示。
 */

#include "main.h"
#include <stdint.h>

/** 上电初始化 OLED。调用一次(放在 main 里 led_init() 附近)。 */
void oled_init(void);

/** 清屏。 */
void oled_clear(void);

/** 清除某一行的 16 个字符格(整行写空格)。 */
void oled_clear_line(uint8_t Line);

/** 在第 Line 行(1~4)、第 Column 列(1~16)显示 ASCII 字符串。 */
void oled_show_string(uint8_t Line, uint8_t Column, const char *String);

/** 在第 Line 行、第 Column 列显示无符号十进制数(Length 位)。 */
void oled_show_num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/** 状态屏: sent=1 → "SEND OK"(发送成功), sent=0 → "NOT SEND"(未发送成功)。 */
void oled_show_status(uint8_t sent);

/** Display the current sorting stage on line 1 while keeping camera details. */
void oled_show_stage(const char *stage);

/** Display camera receive diagnostics after a timeout or invalid result.
 * diag[0..2] = LS bytes/error/parsed, diag[3..5] = YB bytes/error/parsed. */
void oled_show_camera_diagnostics(const int32_t diag[6]);

/**
 * 显示一轮双摄像头识别和融合结果。
 *
 * 由于目标板使用 8x16 字体、屏幕只有四行，内容按下面的布局压缩显示：
 *   第 1 行: LS 原始形状
 *   第 2 行: YB 原始形状 + 颜色
 *   第 3 行: F 融合形状 + 颜色
 *   第 4 行: R 融合规则原因
 * shape/color 传 -1 表示 unknown；reason 可为 NULL。
 */
void oled_show_cameras(int32_t shape0, int32_t shape1, int32_t color1,
                       int32_t fused_shape, int32_t fused_color,
                       const char *reason);

#endif /* __OLED_APP_H */
