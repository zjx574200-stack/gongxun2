/**
 * @file    servo_port.c
 * @brief   PWM 舵机硬件层实现（STM32 HAL 库，TIM4，兼容 F103/F407）
 *
 * 通道映射（本工程分配 2 路，均为空闲引脚）：
 *   通道0 → TIM4 CH1 → PB6
 *   通道1 → TIM4 CH2 → PB7
 *
 * 本文件是唯一随硬件/库适配的文件，servo.c / servo.h 无需改动。
 * 采用「自包含初始化」：不依赖 CubeMX 生成的 MX_TIMx_Init，无需重新生成 .ioc。
 */
#include "servo.h"
#include "stm32f4xx_hal.h"

/* HAL 头文件选择（与工程 zdt_motor_port.h 约定一致）。
 * ⚠️ 注意: Keil 里定义的是 STM32F407xx, 不是 STM32F4, 所以判断要三者都写全。 */
#if defined(STM32F1)
  #include "stm32f1xx_hal.h"
#elif defined(STM32F4) || defined(STM32F407xx) || defined(STM32F4XX)
  #include "stm32f4xx_hal.h"
#else
  #include "stm32f1xx_hal.h"
#endif

/* ==================== 用户配置区（换芯片/引脚时改这里） ==================== */
#define SERVO_TIM            TIM4
#define SERVO_GPIO_PORT      GPIOB
#define SERVO_GPIO_PIN_CH0   GPIO_PIN_6   /* 通道0 → PB6 */
#define SERVO_GPIO_PIN_CH1   GPIO_PIN_7   /* 通道1 → PB7 */

#if defined(STM32F4) || defined(STM32F407xx) || defined(STM32F4XX)
  /* F407: APB1 定时器 42MHz x2 = 84MHz */
  #define SERVO_TIM_CLK_HZ   84000000UL
#else
  /* F103: APB1 定时器 36MHz x2 = 72MHz */
  #define SERVO_TIM_CLK_HZ   72000000UL
#endif

/* 计数频率设为 1MHz → 1 计数值 = 1us，CCR 值直接就是微秒 */
#define SERVO_TIM_PSC       (SERVO_TIM_CLK_HZ / 1000000UL - 1)
#define SERVO_TIM_PERIOD    (20000UL - 1)   /* 20ms 周期（50Hz） */
/* ============================================================================ */

static TIM_HandleTypeDef htim_servo;

void servo_port_init(void) {
    GPIO_InitTypeDef   gpio = {0};
    TIM_OC_InitTypeDef oc   = {0};

    /* 1) 使能时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 2) GPIO：PB6/PB7 复用推挽输出 */
    gpio.Pin   = SERVO_GPIO_PIN_CH0 | SERVO_GPIO_PIN_CH1;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(STM32F4) || defined(STM32F407xx) || defined(STM32F4XX)
    /* F4 必须显式指定复用功能号: 不设的话 AFR 保持复位值 AF0(系统), TIM4 的 PWM
     * 根本到不了 PB6/PB7 —— 舵机收不到任何波形, 就是"没使能"那种软塌塌的样子。
     * F1 是隐式映射(GPIO_InitTypeDef 也没有 Alternate 字段), 所以用条件编译隔开。 */
    gpio.Alternate = GPIO_AF2_TIM4;
#endif
    HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);

    /* 3) 定时器基配置：1MHz 计数，20ms 周期 */
    htim_servo.Instance               = SERVO_TIM;
    htim_servo.Init.Prescaler         = SERVO_TIM_PSC;
    htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_servo.Init.Period            = SERVO_TIM_PERIOD;
    htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim_servo);    /* 配置时基（内部会调用弱 MspInit） */

    /* 4) 通道 PWM 配置：初始中位 1500us */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = SERVO_PWM_MID_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim_servo, &oc, TIM_CHANNEL_2);

    /* 5) 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_servo, TIM_CHANNEL_2);
}

void servo_port_write(uint8_t channel, uint16_t us) {
    switch (channel) {
        case 0: __HAL_TIM_SET_COMPARE(&htim_servo, TIM_CHANNEL_1, us); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim_servo, TIM_CHANNEL_2, us); break;
        default: break;   /* 超出 SERVO_MAX_CHANNEL，忽略 */
    }
}

uint16_t servo_port_read(uint8_t channel) {
    switch (channel) {
        case 0: return (uint16_t)__HAL_TIM_GET_COMPARE(&htim_servo, TIM_CHANNEL_1);
        case 1: return (uint16_t)__HAL_TIM_GET_COMPARE(&htim_servo, TIM_CHANNEL_2);
        default: return 0;
    }
}
