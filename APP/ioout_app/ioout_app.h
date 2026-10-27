/**
 * 1. 使用此模块前，必须在STM32CubeMX中将所需引脚配置为GPIO_Output模式。
 * 2. 本模块最大支持8个GPIO输出通道 (0-7)。
 * 3. 所有函数都是非阻塞的。
 */

#ifndef __IOOUT_APP_H__
#define __IOOUT_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mydefine.h"

/* Defines -------------------------------------------------------------------*/
#define GPIO_OUT_CHANNEL_MAX      8   // 定义最大支持的输出通道数量

/* Enums ---------------------------------------------------------------------*/
/**
 * @brief GPIO输出电平状态枚举
 */
typedef enum
{
    GPIO_OUT_STATE_LOW  = 0, // 低电平
    GPIO_OUT_STATE_HIGH = 1  // 高电平
} GpioOut_State_e;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief  初始化GPIO输出模块。
 * @note   此函数应在系统初始化时调用一次，它会将所有通道重置为禁用状态。
 * @param  None
 * @retval None
 */
void GpioOut_Init(void);

/**
 * @brief  配置并启用一个GPIO输出通道。
 * @note   将一个物理GPIO引脚与一个逻辑通道号绑定。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @param  port: GPIO端口 (例如 GPIOA, GPIOB)。
 * @param  pin: GPIO引脚 (例如 GPIO_PIN_0, GPIO_PIN_13)。
 * @retval None
 */
void GpioOut_ConfigureChannel(uint8_t channel, GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief  禁用一个GPIO输出通道。
 * @note   禁用后，对此通道的任何操作都将无效。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @retval None
 */
void GpioOut_DisableChannel(uint8_t channel);

/**
 * @brief  设置指定通道的输出电平。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @param  state: 要设置的电平状态, GPIO_OUT_STATE_LOW 或 GPIO_OUT_STATE_HIGH。
 * @retval None
 */
void GpioOut_SetState(uint8_t channel, GpioOut_State_e state);

/**
 * @brief  设置指定通道输出为高电平。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @retval None
 */
void GpioOut_SetHigh(uint8_t channel);

/**
 * @brief  设置指定通道输出为低电平。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @retval None
 */
void GpioOut_SetLow(uint8_t channel);

/**
 * @brief  翻转指定通道的输出电平。
 * @param  channel: 通道号, 范围 [0, GPIO_OUT_CHANNEL_MAX - 1]。
 * @retval None
 */
void GpioOut_Toggle(uint8_t channel);


/**
 * @brief  IO口初始化
 * @param  None
 * @retval None
 */
void ioout_init(void);

/**
 * @brief  LED相关任务
 * @param  None
 * @retval None
 */
void led_task(void);


#ifdef __cplusplus
}
#endif

#endif /* __GPIO_OUT_H__ */

