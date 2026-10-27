#ifndef __SCREEN_APP_H
#define __SCREEN_APP_H

/**
 * @file    screen_app.h
 * @brief   淘晶驰(TJC)串口屏模块 (F407) —— 头文件
 *
 * 屏型号: TJC4827T143_011 (4.3寸 272×480 竖屏), 屏内工程 test_v2.HMI, baud=115200。
 * 本板直连屏幕(不再像 F103 版那样经二号板透传):
 *   屏 RX ← PC6 (USART6_TX),  屏 TX → PC7 (USART6_RX),  GND 共地。
 * 协议: 每条指令 = ASCII 字符串 + 0xFF 0xFF 0xFF 帧尾。
 * 本模块只用 TX 单向发指令(纯显示), 不开 RX 中断。
 *
 * 屏上是一张 7 行成绩表(行号 = 控件名后缀 t0~t6 / p0~p6 / n0~n6, 每个 t/p/n = 货名/图片/计数):
 *   每行内容不按形状固定, 而是按"识别到的先后顺序"动态占序号——
 *   首次出现的 (形状,颜色) 组合占下一个空序号(序号1→行0, 序号2→行1, …)。
 *   行 0~5 = 6 个动态序号(转盘 6 格); 最后一行(行6) = "Total" 总计(所有序号累计件数)。
 * 图片 ID 3~66 = 8 形状列 × 8 颜色(白红黄蓝绿紫橙灰), 见 图片库ID对照表.md:
 *   pic_id = 3 + 形状列×8 + 颜色偏移(白0 红1 黄2 蓝3 绿4 紫5 橙6 灰7);
 *   形状列只由形状决定, 与显示在哪一行无关
 */

#include "main.h"
#include <stdint.h>

/* ---------- API ---------- */

/** 手动初始化 USART6(PC6/PC7, 115200), 上电调用一次。
 *  不经过 CubeMX 的 usart.c, 避免 CubeMX 重新生成时动到 USART1/2/3/UART4 的引脚。 */
void screen_init(void);

/** 发一条原始 TJC 指令, 自动追加 FF FF FF 帧尾。例: screen_send("page 0"); */
void screen_send(const char *fmt, ...);

/** 设置图片控件: p<row>.pic=<pic_id> */
void screen_set_pic(uint8_t row, uint8_t pic_id);

/** 设置数字控件: n<row>.val=<val> */
void screen_set_num(uint8_t row, int32_t val);

/** 设置文本控件: t<row>.txt="<s>" */
void screen_set_text(uint8_t row, const char *s);

/**
 * 识别到 (形状, 颜色) 后调用: 首次遇到的组合占下一个空序号(和转盘格位共用
 * vision_slot_index 那张表, 序号一致), 把对应图片调到该序号那一行,
 * 累加该行计数并更新货名。
 * @param shape  F407 视觉形状编号 0~7 (见 vision_app.h), -1=未知
 * @param color  F407 视觉颜色编号 0~7 (见 vision_app.h), -1=未知
 * @return 1=已显示;  0=形状/颜色非法, 或 6 个序号都用满(转盘/屏共用的格位表满)。
 */
uint8_t screen_show_goods(int32_t shape, int32_t color);

#endif /* __SCREEN_APP_H */
