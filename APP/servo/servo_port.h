/**
 * @file    servo_port.h
 * @brief   PWM 舵机硬件抽象层接口（唯一需要随芯片/库移植的部分）
 *
 * 本层只暴露 3 个函数，上层 servo.c / servo.h 完全与硬件无关。
 * 移植到新芯片（如 STM32F407）时，只需修改 servo_port.c 中
 * 这 3 个函数的实现，servo.c 无需任何改动。
 */
#ifndef __SERVO_PORT_H
#define __SERVO_PORT_H

#include <stdint.h>

/* 上电初始化一次：配置定时器 PWM 与 GPIO 复用 */
void     servo_port_init(void);

/* 写某通道脉宽，us 范围 500~2500（对应 0°~最大角度） */
void     servo_port_write(uint8_t channel, uint16_t us);

/* 读回某通道当前脉宽（可选实现，未实现时返回 0） */
uint16_t servo_port_read(uint8_t channel);

#endif /* __SERVO_PORT_H */
