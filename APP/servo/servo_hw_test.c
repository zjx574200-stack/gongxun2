#include "servo_hw_test.h"
#include "stm32f4xx_hal.h"

#define SERVO_FRAME_US       20000U
#define SERVO_MIN_US           500U
#define SERVO_MAX_US          2500U
#define SERVO_RAMP_FRAMES       50U
#define SERVO_PHASE_FRAMES     100U

/* Keep the calibrated angles in main.c:
 * servo1: 270 degree model, home 95 degrees, open 75 degrees.
 * servo2: 180 degree model, home 150 degrees, open 70 degrees.
 * pulse = 500 + angle * 2000 / maximum_angle, rounded to nearest us. */
#define SERVO1_HOME_US        1204U
#define SERVO1_OPEN_US        1056U
#define SERVO2_HOME_US        2167U
#define SERVO2_OPEN_US        1278U

volatile uint32_t g_servo_test_frames;
volatile uint32_t g_servo_test_phase;
volatile uint16_t g_servo_test_us[2];

static uint32_t phase_start_frame;
static uint32_t last_frame;
static uint16_t phase_start_us[2];

static const uint16_t phase_targets[4][2] = {
    {SERVO1_OPEN_US, SERVO2_HOME_US},
    {SERVO1_HOME_US, SERVO2_HOME_US},
    {SERVO1_HOME_US, SERVO2_OPEN_US},
    {SERVO1_HOME_US, SERVO2_HOME_US}
};

void ServoHwTest_SetPulseUs(uint8_t channel, uint16_t pulse_us)
{
    if (channel >= 2U) return;
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

    /* CCR preload applies changes at the next period boundary. */
    if (channel == 0U) TIM4->CCR1 = pulse_us;
    else TIM4->CCR2 = pulse_us;
    g_servo_test_us[channel] = pulse_us;
}

void ServoHwTest_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    __HAL_RCC_TIM4_FORCE_RESET();
    __HAL_RCC_TIM4_RELEASE_RESET();

    /* At 168 MHz SYSCLK: APB1=42 MHz, TIM4=84 MHz, PSC=83.
     * 1 MHz counter and ARR=19999 give a continuous 20 ms period. */
    TIM4->PSC = timer_clock / 1000000U - 1U;
    TIM4->ARR = SERVO_FRAME_US - 1U;
    TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                   TIM_CCMR1_OC1PE |
                   TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 |
                   TIM_CCMR1_OC2PE;
    TIM4->CR1 = TIM_CR1_ARPE;
    ServoHwTest_SetPulseUs(0U, SERVO1_HOME_US);
    ServoHwTest_SetPulseUs(1U, SERVO2_HOME_US);
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    TIM4->CNT = 0U;

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_servo_test_frames = 0U;
    g_servo_test_phase = 0U;
    phase_start_frame = 0U;
    last_frame = 0U;
    phase_start_us[0] = SERVO1_HOME_US;
    phase_start_us[1] = SERVO2_HOME_US;

    HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 5U, 0U);
    TIM4->DIER = TIM_DIER_UIE;
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/* This IRQ never toggles GPIO, delays, waits, or calls servo movement code. */
void TIM4_IRQHandler(void)
{
    if (((TIM4->SR & TIM_SR_UIF) != 0U) &&
        ((TIM4->DIER & TIM_DIER_UIE) != 0U)) {
        TIM4->SR = ~TIM_SR_UIF;
        g_servo_test_frames++;
    }
}

void ServoHwTest_Task(void)
{
    uint32_t now = g_servo_test_frames;
    uint32_t elapsed;
    uint32_t channel;
    uint32_t phase;
    int32_t start;
    int32_t target;

    if (now == last_frame) return;
    last_frame = now;
    elapsed = now - phase_start_frame;
    if (elapsed >= SERVO_PHASE_FRAMES) {
        phase = g_servo_test_phase + 1U;
        if (phase > 4U) phase = 1U;
        g_servo_test_phase = phase;
        phase_start_frame = now;
        phase_start_us[0] = g_servo_test_us[0];
        phase_start_us[1] = g_servo_test_us[1];
        elapsed = 0U;
    }
    phase = g_servo_test_phase;
    if (phase == 0U) return;
    if (elapsed > SERVO_RAMP_FRAMES) elapsed = SERVO_RAMP_FRAMES;

    /* Move for one second, then hold for one second, without HAL_Delay.
     * Integer arithmetic also keeps this test independent of the FPU. */
    for (channel = 0U; channel < 2U; channel++) {
        start = phase_start_us[channel];
        target = phase_targets[phase - 1U][channel];
        ServoHwTest_SetPulseUs((uint8_t)channel,
            (uint16_t)(start + (target - start) * (int32_t)elapsed /
                                    (int32_t)SERVO_RAMP_FRAMES));
    }
}
