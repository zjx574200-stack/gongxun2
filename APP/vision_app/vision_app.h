#ifndef __VISION_APP_H
#define __VISION_APP_H

/**
 * @file    vision_app.h
 * @brief   双 K230 摄像头视觉融合模块 (F407)
 *
 * 触发: 分拣主流程让二级传送带运行固定时间后, 向两台摄像头各发 "1\n"(0x31 0x0A)。
 *       摄像头收到 "1\n" 开始识别, 各自回一包结果。
 * 结果帧: 帧头 '@', 帧尾回车(0x0D), 中间用英文逗号分隔:
 *   庐山(UART5): "@形状,数值"
 *   亚博(USART2): "@形状,颜色,数值"
 *   - 形状(八分类): cone / cube / cuboid / cylinder / hexagonal_prism /
 *     pentagonal_prism / triangular_prism / triangular_pyramid; unknown = 未检测到。
 *   - 颜色(八分类): yellow / blue / green / white / red / purple / orange / gray;
 *     unknown = 未知(只有亚博回)。
 *   - 数值: 识别置信度, 用于两台形状不一致时取高者。
 * 融合: 按源工程的十条规则处理两路结果，并保留原始结果、融合分数和规则原因供 OLED/调试。
 *
 * 接线: 庐山 TX→PD2(UART5_RX), RX→PC12(UART5_TX);
 *       亚博 TX→PA3(USART2_RX), RX→PA2(USART2_TX); 115200 8N1, 共地。
 */

#include "main.h"
#include <stdint.h>

/* 转盘每格角度 (6 格 = 360°)。若实测每格不是 60°(齿轮比/机械偏差),
 * 按「实测每格角度」微调这个常数。 */
#define VISION_SLOT_ANGLE_DEG   60.0f

/* 转盘初始位置标定偏移(度): 转盘"机械零点" 与 六格实际中心 之间的固定角度差。 */
#define VISION_ANGLE_OFFSET_DEG   -25.0f

/* 最新识别出的形状: 0~7, -1 = 未知/还没收到有效结果 */
extern volatile int32_t g_vision_shape;

/* 最新融合出的颜色: 0~7, -1 = 未知/还没收到 */
extern volatile int32_t g_vision_color;

/* 最新融合分数: 同类取两路较低值，其余取获胜一路的置信度。 */
extern volatile float g_vision_score;

/* 本轮命中的融合规则原因，供 OLED/调试显示。 */
extern volatile const char *g_vision_reason;

/* 两路原始结果(未融合): shape0=庐山, shape1/color1=亚博。 */
extern volatile int32_t g_vision_shape0;
extern volatile int32_t g_vision_shape1;
extern volatile int32_t g_vision_color1;

/* 新结果标志: 1=有新结果(待消费) 0=已消费 */
extern volatile uint8_t  g_vision_new_frame;

/* 累计收到的字节数(两路合计, 调试用) */
extern volatile uint32_t g_vision_rx_bytes;

/* ---------- API ---------- */

/** 使能 UART5(庐山)/USART2(亚博) 的 RXNE 接收中断并初始化融合状态。上电调用一次。 */
void vision_init(void);

/** RXNE 接收中断处理: 在 stm32f4xx_it.c 的 UART5/USART2 IRQ 里调用。
 *  读 SR/DR 清 RXNE, 把字节压进对应串口的环形缓冲。 */
void vision_uart_irq(UART_HandleTypeDef *huart);

/** 阻塞一轮识别: 向两台摄像头发 "1\n" → 收两路 "@形状,颜色\r" → 融合。
 *  成功返回 0(结果在 g_vision_shape/g_vision_color), 超时返回 -1。 */
int32_t vision_capture(uint32_t timeout_ms);

/** 形状+颜色 → 动态格位序号(1~6)。首次遇到的组合占下一个空格; 已见过返回原序号;
 *  形状/颜色非法返回 0; 6 格用满后新类别沿用 1 号格。屏上序号和转盘格位共用这张表。 */
int32_t vision_slot_index(int32_t shape, int32_t color);

/** 形状+颜色 → 转盘格角度。两者完全一致才同格, 动态分配; 非法返回 0°。 */
float vision_slot_angle(int32_t shape, int32_t color);

/** 调试: 形状/颜色编号 → 名字 */
const char *vision_shape_name(int32_t shape);
const char *vision_color_name(int32_t color);

/** 调试: 形状编号 → 短名(≤6 字符, OLED 显示用)。 */
const char *vision_shape_short(int32_t shape);

/** 单台摄像头最近一轮识别结果快照(调试/OLED 显示用)。 */
typedef struct {
    int32_t  shape;    /* 0~7, -1 = 未知/没认出 */
    int32_t  color;    /* 0~7, -1 = 未知(庐山永远 -1) */
    float    score;    /* 识别置信度数值 */
    uint8_t  got;      /* 本轮是否收到有效回包 */
    uint32_t rx_bytes; /* 本轮累计收到的字节数(含坏字节), 判断"到底有没有数据回来" */
    uint8_t  rx_error; /* 本轮是否出现帧错误(ORE/NE/FE/PE), 1=波特率/接反/共地问题 */
} VisionCamInfo;

/** 读取某台摄像头(port0=庐山, port1=亚博) 最近一轮的结果快照。 */
void vision_get_cam(uint8_t port, VisionCamInfo *out);

/** 本轮累计发给庐山(UART5)的 "1\n" 触发次数(每发一次 +1, 每轮识别开始清零)。 */
extern volatile uint32_t g_vision_sent_ls;

/** 读取最近一轮两路接收诊断:
 * out[0..2]=庐山字节数/帧错误/是否解析成功;
 * out[3..5]=亚博字节数/帧错误/是否解析成功。 */
void vision_debug_get(int32_t out[6]);

#endif /* __VISION_APP_H */
