#ifndef __CONVEYOR_APP_H
#define __CONVEYOR_APP_H

#include "zdt_motor/zdt_motor.h"

/* Keep these values equal to the IDs stored in the two drive controllers. */
#define ZDT_CONVEYOR_1_MOTOR_ID      2U
#define ZDT_CONVEYOR_2_MOTOR_ID      3U

/* Belt direction and speed can be tuned without touching protocol code. */
#define CONVEYOR_1_DIRECTION         ZDT_DIR_CCW
#define CONVEYOR_2_DIRECTION         ZDT_DIR_CCW
#define CONVEYOR_1_SPEED_RPM         15.0f
#define CONVEYOR_2_SPEED_RPM         15.0f
#define CONVEYOR_ACCEL               10U
#define CONVEYOR_COMMAND_GAP_MS      5U

#if (ZDT_CONVEYOR_1_MOTOR_ID < 1U) || (ZDT_CONVEYOR_1_MOTOR_ID > 247U) || \
    (ZDT_CONVEYOR_2_MOTOR_ID < 1U) || (ZDT_CONVEYOR_2_MOTOR_ID > 247U)
#error "ZDT conveyor motor IDs must be in the range 1..247"
#endif

#if (ZDT_CONVEYOR_1_MOTOR_ID == ZDT_CONVEYOR_2_MOTOR_ID)
#error "The two ZDT conveyor motor IDs must be different"
#endif

typedef enum {
    CONVEYOR_1 = 0,
    CONVEYOR_2,
    CONVEYOR_COUNT
} ConveyorId_t;

/* Call after GPIO, UART and DMA have been initialized. */
void conveyor_init(void);

/* Call from the main loop or a state machine, never from an IRQ handler. */
void conveyor_start(ConveyorId_t conveyor);
void conveyor_stop(ConveyorId_t conveyor);
void conveyor_stop_all(void);

#endif /* __CONVEYOR_APP_H */
