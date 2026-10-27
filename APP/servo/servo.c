/**
 * @file    servo.c
 * @brief   PWM 舵机通用驱动层实现（纯逻辑，与硬件无关）
 *
 * 本文件不直接操作任何寄存器/外设，所有硬件访问都通过 servo_port.c 完成，
 * 因此跨芯片（F103/F407）移植时本文件无需改动。
 */
#include "servo.h"
#include <string.h>

/* ==================== 角度 <-> 脉宽 换算 ==================== */

uint16_t Servo_AngleToUs(float angle, float max_angle) {
    float us = SERVO_PWM_MIN_US
             + angle * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) / max_angle;
    if (us < SERVO_PWM_MIN_US) us = SERVO_PWM_MIN_US;
    if (us > SERVO_PWM_MAX_US) us = SERVO_PWM_MAX_US;
    return (uint16_t)(us + 0.5f);
}

float Servo_UsToAngle(uint16_t us, float max_angle) {
    return (float)(us - SERVO_PWM_MIN_US)
         * max_angle / (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US);
}

/* ==================== 初始化 ==================== */

void Servo_HW_Init(void) {
    servo_port_init();
}

void Servo_Init(Servo_HandleTypeDef *h, uint8_t channel, float max_angle) {
    memset(h, 0, sizeof(Servo_HandleTypeDef));
    h->channel   = channel;
    h->max_angle = max_angle;
    h->cur_us    = SERVO_PWM_MID_US;
    h->cur_us_f  = SERVO_PWM_MID_US;
    h->target_us = SERVO_PWM_MID_US;
    Servo_SetUs(h, SERVO_PWM_MID_US);   /* 上电居中，避免装配后乱跳 */
}

/* ==================== 控制 ==================== */

void Servo_SetUs(Servo_HandleTypeDef *h, uint16_t us) {
    if (us < SERVO_PWM_MIN_US) us = SERVO_PWM_MIN_US;
    if (us > SERVO_PWM_MAX_US) us = SERVO_PWM_MAX_US;

    h->cur_us    = us;
    h->cur_us_f  = (float)us;
    h->target_us = us;
    h->step_us   = 0.0f;
    h->moving    = 0;

    servo_port_write(h->channel, us);
}

void Servo_SetAngle(Servo_HandleTypeDef *h, float angle) {
    Servo_SetUs(h, Servo_AngleToUs(angle, h->max_angle));
}

void Servo_MoveTo(Servo_HandleTypeDef *h, float angle, uint16_t time_ms) {
    uint16_t target = Servo_AngleToUs(angle, h->max_angle);

    if (time_ms == 0 || target == h->cur_us) {
        Servo_SetUs(h, target);   /* 时间 0 或位置不变 -> 立即到位 */
        return;
    }

    h->target_us = target;
    h->step_us   = ((float)target - h->cur_us_f) / (float)time_ms;  /* us/ms */
    h->moving    = (h->step_us != 0.0f);
}

void Servo_Stop(Servo_HandleTypeDef *h) {
    h->moving  = 0;
    h->step_us = 0.0f;   /* 保持当前位置不动 */
}

void Servo_Update(Servo_HandleTypeDef *h) {
    if (!h->moving) return;

    h->cur_us_f += h->step_us;

    if (h->step_us > 0) {
        if (h->cur_us_f >= h->target_us) {
            h->cur_us_f = h->target_us;
            h->moving   = 0;
        }
    } else {
        if (h->cur_us_f <= h->target_us) {
            h->cur_us_f = h->target_us;
            h->moving   = 0;
        }
    }

    h->cur_us = (uint16_t)(h->cur_us_f + 0.5f);
    servo_port_write(h->channel, h->cur_us);
}

void Servo_UpdateAll(Servo_HandleTypeDef *hs, uint8_t num) {
    uint8_t i;
    for (i = 0; i < num; i++) {
        Servo_Update(&hs[i]);
    }
}

/* ==================== 查询 ==================== */

float Servo_GetAngle(Servo_HandleTypeDef *h) {
    return Servo_UsToAngle(h->cur_us, h->max_angle);
}

uint8_t Servo_IsMoving(Servo_HandleTypeDef *h) {
    return h->moving;
}
