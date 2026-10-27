#include "conveyor_app.h"

#include "main.h"
#include "zdt_motor/zdt_motor_port.h"

static zdt_t conveyor_motors[CONVEYOR_COUNT];

static const uint8_t conveyor_motor_ids[CONVEYOR_COUNT] = {
    ZDT_CONVEYOR_1_MOTOR_ID,
    ZDT_CONVEYOR_2_MOTOR_ID
};

static const zdt_dir_t conveyor_directions[CONVEYOR_COUNT] = {
    CONVEYOR_1_DIRECTION,
    CONVEYOR_2_DIRECTION
};

static const float conveyor_speeds_rpm[CONVEYOR_COUNT] = {
    CONVEYOR_1_SPEED_RPM,
    CONVEYOR_2_SPEED_RPM
};

/* F407 uses the USART3 conveyor bus; the same source remains usable on F103. */
#if defined(STM32F4) || defined(STM32F407xx) || defined(STM32F4XX)
#define CONVEYOR_SEND_FN zdt_port3_send
#else
#define CONVEYOR_SEND_FN zdt_port_send
#endif

static bool conveyor_is_valid(ConveyorId_t conveyor)
{
    return ((uint32_t)conveyor < (uint32_t)CONVEYOR_COUNT);
}

void conveyor_init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)CONVEYOR_COUNT; i++) {
        zdt_init(&conveyor_motors[i], ZDT_FW_EMM,
                 conveyor_motor_ids[i], CONVEYOR_SEND_FN);
    }

    /* Give both drive controllers time to finish their power-up sequence. */
    HAL_Delay(500U);

    /* Clear a latched stall-protection state before enabling.  The EMM driver
     * can go offline after a stall; an enable frame alone is then ignored. */
    for (i = 0U; i < (uint8_t)CONVEYOR_COUNT; i++) {
        zdt_reset_clog(&conveyor_motors[i]);
        HAL_Delay(CONVEYOR_COMMAND_GAP_MS);
        zdt_enable(&conveyor_motors[i], true);
        HAL_Delay(CONVEYOR_COMMAND_GAP_MS);
        zdt_stop(&conveyor_motors[i]);
        HAL_Delay(CONVEYOR_COMMAND_GAP_MS);
    }
}

void conveyor_start(ConveyorId_t conveyor)
{
    if (!conveyor_is_valid(conveyor)) {
        return;
    }

    zdt_run_vel(&conveyor_motors[conveyor],
                conveyor_directions[conveyor],
                CONVEYOR_ACCEL,
                conveyor_speeds_rpm[conveyor]);
    HAL_Delay(CONVEYOR_COMMAND_GAP_MS);
}

void conveyor_stop(ConveyorId_t conveyor)
{
    if (!conveyor_is_valid(conveyor)) {
        return;
    }

    zdt_stop(&conveyor_motors[conveyor]);
    HAL_Delay(CONVEYOR_COMMAND_GAP_MS);
}

void conveyor_stop_all(void)
{
    conveyor_stop(CONVEYOR_1);
    conveyor_stop(CONVEYOR_2);
}
