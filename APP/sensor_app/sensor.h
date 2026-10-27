#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f4xx_hal.h"

/********************* 初始化 *********************/
void Sensor_Init(void);

/********************* 轮询更新（主循环每轮调用1次） *********************/
void Sensor_Update(void);

/********************* 业务事件查询（main里直接用） *********************/
// 中间传感器：检测到物料进入（上升沿/下降沿触发，一次只返回一次有效）
uint8_t Sensor_Is_Goods_Coming(void);

// 料盒传感器：检测到物料成功掉落到位
uint8_t Sensor_Is_Goods_Dropped(void);

/********************* 事件标志手动清零（可选） *********************/
void Sensor_Clear_Flags(void);

#endif
