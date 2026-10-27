#include "turntable_app.h"

#include "main.h"
#include "zdt_motor/zdt_motor_port.h"

static zdt_t turntable_motor;

/* Last absolute target (degrees), for computing the travel distance of the next
 * move. homing() resets it to 0, so the wait time is correct on every move. */
static float turntable_last_deg = 0.0f;

volatile int32_t turntable_debug_cpos = 0;
volatile int32_t turntable_debug_fwd_cpos = 0;
volatile int32_t turntable_debug_perr = 0;
volatile uint8_t  turntable_debug_flag = 0;
volatile uint8_t  turntable_debug_origin_flag = 0xFF;

bool turntable_wait_boot_home(void)
{
    uint32_t start = HAL_GetTick();
    bool saw_homing = false;

    zdt_init(&turntable_motor, ZDT_FW_EMM,
             ZDT_TURNTABLE_MOTOR_ID, zdt_port4_send);

    while ((HAL_GetTick() - start) < TURNTABLE_BOOT_HOME_TIMEOUT_MS) {
        const uint8_t *buf;
        uint16_t len;
        uint8_t flag;

        zdt_port_rx_clear(ZDT_BUS_CHASSIS);
        zdt_read_raw(&turntable_motor, ZDT_EMM_RD_OFLAG);
        len = zdt_port_wait_frame(ZDT_BUS_CHASSIS, 30U);
        buf = zdt_port_rx_data(ZDT_BUS_CHASSIS);
        if (len >= 4U && buf[0] == ZDT_TURNTABLE_MOTOR_ID &&
            buf[1] == ZDT_EMM_RD_OFLAG && buf[3] == 0x6BU) {
            flag = buf[2];
            turntable_debug_origin_flag = flag;
            if ((flag & 0x38U) != 0U) return false;
            if ((flag & 0x04U) != 0U) {
                saw_homing = true;
            } else if (saw_homing && (flag & 0x03U) == 0x03U) {
                return true;
            }
        }
        HAL_Delay(20U);
    }
    return false;
}

/* F407 版: 转盘挂在底盘总线(UART4)上, 用 zdt_port4_send 发送、ZDT_BUS_CHASSIS 读回。 */
void turntable_init(void)
{
    zdt_init(&turntable_motor, ZDT_FW_EMM,
             ZDT_TURNTABLE_MOTOR_ID, zdt_port4_send);
    zdt_set_pulses_per_rev(&turntable_motor, TURNTABLE_PULSES_PER_REV);

    /* Covers the driver's power-up time, same as conveyor_init(). */
    HAL_Delay(500U);

    /* 先清掉上次遗留的堵转/堵转保护故障。堵转保护触发后驱动器会"自动关闭"
     * (脱机), 此时直接发使能会被拒 —— 所以必须先解除堵转, 再使能。 */
    zdt_reset_clog(&turntable_motor);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);

    zdt_enable(&turntable_motor, true);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
    zdt_stop(&turntable_motor);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);

    /* Raise the closed-loop (FOC) current so the start-up ramp has enough
     * torque for the turntable's inertia. Saved to driver flash so it also
     * survives a driver power cycle. */
    zdt_set_foc_ma(&turntable_motor, true, TURNTABLE_FOC_MA);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);

    /* 校验使能是否真的生效: 读回状态标志, Ens_TF(bit0) 应为 1。
     * 若还是没置位, 再补一次解除堵转 + 使能。 */
    if ((turntable_read_flag() & TURNTABLE_FLAG_ENABLE) == 0) {
        zdt_reset_clog(&turntable_motor);
        HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
        zdt_enable(&turntable_motor, true);
        HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
        turntable_read_flag();
    }

    /* No zero_pos() here — the origin is established once, by
     * turntable_homing(). That is the single place the software origin
     * gets set, so there is no second reference to drift against. */
}

/* There is no position-feedback "arrived" check yet, so estimate how long the
 * move takes at constant speed and add a fixed margin for the accel/decel ramp. */
static uint32_t turntable_travel_time_ms(float deg)
{
    float travel_ms = deg * 1000.0f / (6.0f * TURNTABLE_SPEED_RPM);
    return (uint32_t)travel_ms + TURNTABLE_SETTLE_MARGIN_MS;
}

/* After confirmed automatic homing, clear the position counter at the saved
 * origin. Do not send the command that changes the driver's calibrated origin. */
void turntable_homing(void)
{
    zdt_zero_pos(&turntable_motor);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
    turntable_last_deg = 0.0f;

    /* After setting the origin here, the driver's position counter should read
     * ~0. This also proves the RS485 readback link works before any move. */
    turntable_debug_cpos = turntable_read_cpos();
}

void turntable_rotate_to(float target_deg)
{
    /* 距离按实际走过的路程算(回程 45°→0° 仍是走 45°); 方向按目标相对当前位置:
     * 角度增大→正转(CW), 角度减小(回 0°)→反转(CCW)。 */
    float dist = (target_deg >= turntable_last_deg)
                 ? target_deg - turntable_last_deg
                 : turntable_last_deg - target_deg;
    zdt_dir_t dir = (target_deg >= turntable_last_deg)
                    ? TURNTABLE_DIRECTION_FORWARD
                    : TURNTABLE_DIRECTION_BACKWARD;

    zdt_move_traj(&turntable_motor, dir,
                  TURNTABLE_ACCEL, TURNTABLE_ACCEL,
                  TURNTABLE_SPEED_RPM, target_deg, ZDT_REF_ZERO);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);

    /* Block until the move should be finished, so callers can safely chain
     * another zdt_move_traj() right after without cutting this one short. */
    HAL_Delay(turntable_travel_time_ms(dist));

    turntable_last_deg = target_deg;
    turntable_debug_cpos = turntable_read_cpos();

    /* Keep the outgoing (non-zero target) reading so it survives the auto-return. */
    if (target_deg != 0.0f) {
        turntable_debug_fwd_cpos = turntable_debug_cpos;
    }
}

void turntable_rotate_and_return(float deg, uint32_t hold_ms)
{
    turntable_rotate_to(deg);
    HAL_Delay(hold_ms);
    turntable_rotate_to(0.0f);
}

void turntable_stop(void)
{
    zdt_stop(&turntable_motor);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
}

/* Read a 32-bit signed parameter (CPOS / PERR) via the Emm read command.
 * Emm 固件读命令是 [addr][cmd][0x6B], 应答 [addr][cmd][d3 d2 d1 d0][0x6B] (7 字节,
 * 数据是大端 int32, 符号已在最高位里); 老 Emm_V5 模板里见过 8 字节带独立符号字节
 * 的变体 [addr][cmd][sign][d3 d2 d1 d0][0x6B], 这里两种都兼容。 */
static int32_t turntable_read_s32(uint8_t cmd)
{
    const uint8_t *buf;
    uint16_t len;
    uint32_t v;
    int32_t  r;
    int      sign;

    zdt_port_rx_clear(ZDT_BUS_CHASSIS);
    zdt_read_raw(&turntable_motor, cmd);
    HAL_Delay(30U);   /* wait for the driver's reply on the RS485 bus */

    buf = zdt_port_rx_data(ZDT_BUS_CHASSIS);
    len = zdt_port_rx_len(ZDT_BUS_CHASSIS);

    if (len >= 8U && buf[7] == 0x6B) {
        /* 8-byte: data at [3..6], sign at [2] (00=正 01=负) */
        v = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
            ((uint32_t)buf[5] << 8)  |  (uint32_t)buf[6];
        sign = (buf[2] == 0x01) ? -1 : 1;
    } else if (len >= 7U && buf[6] == 0x6B) {
        /* 7-byte: data at [2..5], no sign byte (int32 符号在最高位) */
        v = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) |
            ((uint32_t)buf[4] << 8)  |  (uint32_t)buf[5];
        sign = 1;
    } else {
        return -1;
    }

    r = (int32_t)v;
    return (sign < 0) ? -r : r;
}

int32_t turntable_read_cpos(void)
{
    int32_t v = turntable_read_s32(ZDT_EMM_RD_POS);
    if (v != -1) {
        turntable_debug_cpos = v;
    }
    return v;
}

/* 位置误差 = 目标 - 实时 (闭环跟踪偏差)。刚走完一读:接近 0 说明到位,明显非零
 * 说明没到目标(丢步/堵转未到位)。单位按 X42S 手册是 0..65535/圈(×360/65536=度),
 * 若驱动器实际回的是脉冲数则另算——反正「小≈到位,大≈没到位」。 */
int32_t turntable_read_perr(void)
{
    int32_t v = turntable_read_s32(ZDT_EMM_RD_PERR);
    if (v != -1) {
        turntable_debug_perr = v;
    }
    return v;
}

/* 电机状态标志字节 (X42S 手册 5.5.15), 返回原始字节, 无应答返回 0xFF。
 * 位: 0x01 使能 Ens_TF | 0x02 位置到达 Prf_TF | 0x04 堵转标志 Cgi_TF
 *     0x08 堵转保护标志 Cgp_TF | 0x80 掉电 Oac_TF */
uint8_t turntable_read_flag(void)
{
    const uint8_t *buf;
    uint16_t len;

    zdt_port_rx_clear(ZDT_BUS_CHASSIS);
    zdt_read_raw(&turntable_motor, ZDT_EMM_RD_FLAG);
    HAL_Delay(30U);

    buf = zdt_port_rx_data(ZDT_BUS_CHASSIS);
    len = zdt_port_rx_len(ZDT_BUS_CHASSIS);
    if (len < 4U) {
        turntable_debug_flag = 0xFF;   /* 无应答: 调试变量也置 0xFF, 别留着旧值误导 */
        return 0xFF;
    }
    /* Reply: [addr][0x3A][flag][0x6B] — flag byte at buf[2]. */
    turntable_debug_flag = buf[2];
    return buf[2];
}

/* Turn `deg` degrees relative to the CURRENT position (ZDT_REF_REL), so it
 * works without homing/origin. Positive = forward, negative = backward. */
void turntable_rotate_rel(float deg)
{
    zdt_dir_t dir = (deg >= 0.0f) ? TURNTABLE_DIRECTION_FORWARD
                                  : TURNTABLE_DIRECTION_BACKWARD;
    float mag = (deg >= 0.0f) ? deg : -deg;

    zdt_move_traj(&turntable_motor, dir, TURNTABLE_ACCEL, TURNTABLE_ACCEL,
                  TURNTABLE_SPEED_RPM, mag, ZDT_REF_REL);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);

    /* Block until the move should be finished, so the next command doesn't cut
     * this one short. */
    HAL_Delay(turntable_travel_time_ms(mag));

    turntable_debug_cpos = turntable_read_cpos();
    turntable_debug_fwd_cpos = turntable_debug_cpos;
    turntable_read_perr();   /* 位置误差:非零=没到位(丢步/堵转) */
    turntable_read_flag();   /* 状态标志:bit2堵转 / bit3堵转保护 */
}

/* One full revolution → pause TURNTABLE_SPIN_PAUSE_MS → another revolution,
 * forever. Blocking: never returns. */
void turntable_spin_loop(void)
{
    while (1) {
        turntable_rotate_rel(360.0f);
        HAL_Delay(TURNTABLE_SPIN_PAUSE_MS);
    }
}

/* Lean setup for "no driver init": code-side struct only, zero bytes on the bus.
 * The 500 ms just lets the driver finish powering up before the first move frame. */
void turntable_config_only(void)
{
    zdt_init(&turntable_motor, ZDT_FW_EMM,
             ZDT_TURNTABLE_MOTOR_ID, zdt_port4_send);
    zdt_set_pulses_per_rev(&turntable_motor, TURNTABLE_PULSES_PER_REV);
    HAL_Delay(500U);
}

/* Relative mode: command +360° from the motor's CURRENT position. Pure command
 * send, no waiting for the move to finish — the loop cadence (one rev per 2 s)
 * lives in main's HAL_Delay(2000). */
void turntable_spin_one_turn(void)
{
    zdt_move_traj(&turntable_motor, TURNTABLE_DIRECTION_FORWARD,
                  TURNTABLE_ACCEL, TURNTABLE_ACCEL,
                  TURNTABLE_SPEED_RPM, 360.0f, ZDT_REF_REL);
    HAL_Delay(TURNTABLE_COMMAND_GAP_MS);
}

/* 正转 120° → 停 3 秒 → 反转 120° → 停 3 秒, 无限循环。
 * 相对模式, 无需回零; rotate_rel 每次末尾读回位置, 顺带验证 UART4 读链路。 */
void turntable_back_forth_loop(void)
{
    while (1) {
        turntable_rotate_rel(TURNTABLE_BACK_FORTH_ANGLE_DEG);
        HAL_Delay(TURNTABLE_BACK_FORTH_PAUSE_MS);
        turntable_rotate_rel(-TURNTABLE_BACK_FORTH_ANGLE_DEG);
        HAL_Delay(TURNTABLE_BACK_FORTH_PAUSE_MS);
    }
}
