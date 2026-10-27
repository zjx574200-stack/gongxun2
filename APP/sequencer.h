#ifndef __SEQUENCER_H
#define __SEQUENCER_H

#include "main.h"    // 包含 HAL 库头文件，以便使用 HAL_Delay 等
#include <stdint.h>  // 包含标准整数类型，如 uint8_t

// --- 1. 定义系统的所有状态 ---
typedef enum {
    STATE_FEEDING = 0,    // 进料等待（一级传送带转）
    STATE_TRANSPORT,      // 传送到视觉区（二级传送带转）
    STATE_WAIT_VISION,    // 等待视觉识别结果
    STATE_ROTATE_BOX,     // 转盘寻址（步进电机转）
    STATE_DROP,           // 投放与计数（二级传送带投料）
    STATE_UPDATE_UI       // 刷新屏幕UI
} SystemState_t;

// --- 2. 全局标志位声明 (使用 extern，让中断函数能修改它们) ---
extern uint8_t Flag_Item_Detected; // 光电传感器检测标志位
extern uint8_t Flag_Vision_Done;   // 视觉识别完成标志位
extern uint8_t Vision_Result_ID;   // 识别出的货物类别（1-6）

// --- 3. 核心功能函数声明 ---
void Sequencer_Run(void);

#endif /* __SEQUENCER_H */
