/**
 * @file    servo.h
 * @brief   PWM 舵机通用驱动层（与硬件无关，F103/F407 通用）
 *
 * 用法：
 *   1) 确认 servo_port.c 里定时器/引脚与你硬件一致（本工程用 TIM4：PB6/PB7）；
 *   2) 上电先调用 Servo_HW_Init()；
 *   3) 定义 Servo_HandleTypeDef 句柄，用 Servo_Init() 初始化；
 *   4) 用 Servo_SetAngle() 立即转，或用 Servo_MoveTo() 平滑转；
 *   5) 平滑运动需周期调用 Servo_Update()（放 1ms 中断，或 while 里配合 HAL_Delay(1)）。
 *
 * 本文件跨 F103 / F407 通用，移植无需修改。
 */
#ifndef __SERVO_H
#define __SERVO_H

#include "servo_port.h"

/* ==================== 用户配置区 ==================== */
/* 通道数：本工程分配 2 路（TIM4_CH1=PB6、TIM4_CH2=PB7）。
 * 需更多路时改这里，并同步改 servo_port.c 的映射。 */
#define SERVO_MAX_CHANNEL   2

/* 脉宽边界，PM 系列标准值一般无需修改 */
#define SERVO_PWM_MIN_US    500    /* 0°      */
#define SERVO_PWM_MAX_US    2500   /* 最大角度 */
#define SERVO_PWM_MID_US    1500   /* 中位    */
/* ==================================================== */

/* 舵机句柄 */
typedef struct {
    uint8_t  channel;       /* 通道号 0 .. SERVO_MAX_CHANNEL-1 */
    float    max_angle;     /* 该舵机最大角度（180 或 270，按型号填） */
    uint16_t cur_us;        /* 当前脉宽（整数，微秒） */
    float    cur_us_f;      /* 平滑累计（内部使用） */
    uint16_t target_us;     /* 目标脉宽 */
    float    step_us;       /* 平滑步进（us/ms，可为负） */
    uint8_t  moving;        /* 是否正在平滑运动 */
} Servo_HandleTypeDef;

/* ---------- 初始化 ---------- */

/* 硬件初始化（内部调用 servo_port_init，上电调用一次） */
void Servo_HW_Init(void);

/* 初始化舵机句柄：channel 通道号，max_angle 该舵机最大角度 */
void Servo_Init(Servo_HandleTypeDef *h, uint8_t channel, float max_angle);

/* ---------- 控制 ---------- */

/* 立即转到某角度 */
void Servo_SetAngle(Servo_HandleTypeDef *h, float angle);

/* 立即设置脉宽（us，500~2500） */
void Servo_SetUs(Servo_HandleTypeDef *h, uint16_t us);

/* 平滑转到某角度，time_ms 为过渡时间（非阻塞，需配合 Servo_Update） */
void Servo_MoveTo(Servo_HandleTypeDef *h, float angle, uint16_t time_ms);

/* 停止平滑运动，保持当前位置 */
void Servo_Stop(Servo_HandleTypeDef *h);

/* 每 1ms 调用一次，处理平滑运动（可放 SysTick_Handler） */
void Servo_Update(Servo_HandleTypeDef *h);

/* 批量更新多个舵机 */
void Servo_UpdateAll(Servo_HandleTypeDef *hs, uint8_t num);

/* ---------- 查询 ---------- */

/* 读当前角度 */
float Servo_GetAngle(Servo_HandleTypeDef *h);

/* 是否正在运动中（1=运动中，0=已到位） */
uint8_t Servo_IsMoving(Servo_HandleTypeDef *h);

/* ---------- 换算工具 ---------- */

/* 角度 -> 脉宽（max_angle 为 180 或 270） */
uint16_t Servo_AngleToUs(float angle, float max_angle);

/* 脉宽 -> 角度 */
float Servo_UsToAngle(uint16_t us, float max_angle);

#endif /* __SERVO_H */
