#include "sensor.h"

/********************* 硬件引脚（内部使用，外部不可见） *********************/
#define SENSOR_MID_PORT     GPIOG
#define SENSOR_MID_PIN      GPIO_PIN_0

#define SENSOR_BOX_PORT     GPIOG
#define SENSOR_BOX_PIN      GPIO_PIN_1

/********************* 内部状态变量（static 仅本文件可见） *********************/
static uint8_t mid_last_level = 1;    // 上一次中间传感器电平（上拉默认高）
static uint8_t box_last_level = 1;    // 上一次料盒传感器电平

static uint8_t flag_goods_coming = 0; // 物料进入事件标志
static uint8_t flag_goods_dropped = 0;// 物料掉落事件标志

/********************* 内部工具：带防抖读电平 *********************/
static uint8_t read_level(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t cnt = 0;
    for(uint8_t i = 0; i < 5; i++)
    {
        if(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
            cnt++;
        HAL_Delay(1);
    }
    return cnt >= 3 ? 0 : 1; // 0=遮挡(低), 1=无物料(高)
}

/********************* 初始化 *********************/
void Sensor_Init(void)
{
    // 上电先读一次初始电平
    mid_last_level = read_level(SENSOR_MID_PORT, SENSOR_MID_PIN);
    box_last_level = read_level(SENSOR_BOX_PORT, SENSOR_BOX_PIN);
    flag_goods_coming = 0;
    flag_goods_dropped = 0;
}

/********************* 主循环轮询（每轮调用一次） *********************/
void Sensor_Update(void)
{
    uint8_t mid_now = read_level(SENSOR_MID_PORT, SENSOR_MID_PIN);
    uint8_t box_now = read_level(SENSOR_BOX_PORT, SENSOR_BOX_PIN);

    // 中间传感器：高→低 = 物料挡住了（进入）
    if(mid_last_level == 1 && mid_now == 0)
    {
        flag_goods_coming = 1;
    }

    // 料盒传感器：高→低 = 物料掉下来挡住了
    if(box_last_level == 1 && box_now == 0)
    {
        flag_goods_dropped = 1;
    }

    // 更新上一次电平
    mid_last_level = mid_now;
    box_last_level = box_now;
}

/********************* 对外：查询物料是否进入 *********************/
uint8_t Sensor_Is_Goods_Coming(void)
{
    if(flag_goods_coming)
    {
        flag_goods_coming = 0; // 读一次就清零，保证单次触发
        return 1;
    }
    return 0;
}

/********************* 对外：查询物料是否掉落到位 *********************/
uint8_t Sensor_Is_Goods_Dropped(void)
{
    if(flag_goods_dropped)
    {
        flag_goods_dropped = 0;
        return 1;
    }
    return 0;
}

/********************* 对外：强制清空所有事件标志 *********************/
void Sensor_Clear_Flags(void)
{
    flag_goods_coming = 0;
    flag_goods_dropped = 0;
}
