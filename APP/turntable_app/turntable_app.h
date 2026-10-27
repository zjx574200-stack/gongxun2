#ifndef __TURNTABLE_APP_H
#define __TURNTABLE_APP_H

#include "zdt_motor/zdt_motor.h"
#include "zdt_motor/zdt_motor_port.h"

/* 转盘电机挂在底盘总线(UART4)上, 硬件 ID = 1。
 * (F103 版是 USART2 + ID1, F407 版搬到 UART4 + ID1, 同一台电机) */
#define ZDT_TURNTABLE_MOTOR_ID       1U

/* Tune these for the actual turntable mechanism/gearing. */
/* 每圈脉冲数:16细分 = 3200脉冲/圈(位置字段按微步计数,不是编码器1600)。
 * 之前误设 1600 → 命令120°实际只转60°(0.5倍);改3200后120°=正好120°(实测确认)。
 * 若仍有偏差,用角度尺量实际转角,按 实际角度/命令角度 比例微调本常数。 */
#define TURNTABLE_PULSES_PER_REV      3200U
#define TURNTABLE_DIRECTION_FORWARD   ZDT_DIR_CW
#define TURNTABLE_DIRECTION_BACKWARD  ZDT_DIR_CCW
#define TURNTABLE_SPEED_RPM           15.0f  /* 相对模式每2秒转360°=180°/s=30RPM;转盘跟不上就把这数调大一点 */
#define TURNTABLE_ACCEL               100U  /* Emm 加减速系数 0~255:0=瞬间冲速/瞬间刹车,重负载会起步打滑+到位溜车(角度越大越偏);越大斜坡越缓、越稳 */
#define TURNTABLE_COMMAND_GAP_MS      5U
#define TURNTABLE_BOOT_HOME_TIMEOUT_MS 60000U
#define TURNTABLE_SETTLE_MARGIN_MS    1000U  /* 没有到位反馈,按估算行走时间等待时额外加的加减速/稳定余量 */
#define TURNTABLE_FOC_MA              2000U  /* 闭环电流 mA,默认约2000;转盘惯量大,默认电流起步扭矩不够会丢步堵转(X42S 建议≤4000) */

/* Single knob for turntable_rotate_and_return(): how far to turn, and how long
 * to sit there before auto-returning. Keep TARGET_ANGLE a multiple of 60,
 * in (0, 360], to match the sorter's 6 bins. */
#define TURNTABLE_TARGET_ANGLE_DEG    360.0f
#define TURNTABLE_RETURN_DELAY_MS     3000U
#define TURNTABLE_SPIN_PAUSE_MS       3000U  /* turntable_spin_loop():每转完一整圈停多久再转下一圈 */
#define TURNTABLE_BACK_FORTH_ANGLE_DEG  120.0f  /* turntable_back_forth_loop():正转/反转的角度 */
#define TURNTABLE_BACK_FORTH_PAUSE_MS   2000U  /* turntable_back_forth_loop():转完停多久再反向 */

/* Require an observed auto-homing start and successful completion. */
bool turntable_wait_boot_home(void);

/* Call after GPIO and the ZDT UART have been initialized. */
void turntable_init(void);

/* Lean setup for "no driver init": sets the code-side struct (address, send
 * callback, pulses-per-rev) WITHOUT sending any RS485 command — relies on the
 * driver config (enable, FOC, …) already saved to flash. Sends zero bytes. */
void turntable_config_only(void);

/* One full revolution (360°) relative to the current position, then readback.
 * No homing/origin required. */
void turntable_spin_one_turn(void);

/* 正转 TURNTABLE_BACK_FORTH_ANGLE_DEG → 停 TURNTABLE_BACK_FORTH_PAUSE_MS → 反转同样角度 → 再停, 循环。
 * 相对模式, 无需回零; 阻塞, 永不返回。 */
void turntable_back_forth_loop(void);

/* Clear the position counter after confirmed driver automatic homing. */
void turntable_homing(void);

/* Last boot homing status; 0xFF means no valid reply. */
extern volatile uint8_t turntable_debug_origin_flag;

/* Move to an absolute angle measured from the origin (ZDT_REF_ZERO). */
void turntable_rotate_to(float target_deg);

/* Go to absolute angle `deg`, wait `hold_ms`, then return to the origin (0°). */
void turntable_rotate_and_return(float deg, uint32_t hold_ms);

/* Turn `deg` degrees relative to the CURRENT position (ZDT_REF_REL) — no
 * homing/origin needed. Positive = forward, negative = backward. Blocks until
 * the move finishes (and reads back position/PERR/FLAG). */
void turntable_rotate_rel(float deg);

/* Turn one full revolution (360°), pause TURNTABLE_SPIN_PAUSE_MS, then turn
 * another — forever. Relative mode, so no homing/origin is required.
 * Blocks: it never returns. */
void turntable_spin_loop(void);

void turntable_stop(void);

/* Read the driver's real-time position (ZDT_EMM_RD_POS, big-endian int32 pulses).
 * Returns the pulse value, or -1 if the driver didn't reply in time. */
int32_t turntable_read_cpos(void);

/* 读位置误差(目标-实时):刚走完一读,接近0=到位,明显非零=没到位(丢步/堵转)。-1=无应答。 */
int32_t turntable_read_perr(void);

/* 读状态标志字节,返回原始字节,无应答返回 0xFF。 */
uint8_t turntable_read_flag(void);

/* Last readback value, refreshed by turntable_read_cpos(). Watch in the debugger. */
extern volatile int32_t turntable_debug_cpos;

/* Position pulses read on the FORWARD (non-zero target) move — survives the auto-return,
 * so after rotate_and_return() you can still see what the table reached at the target.
 * Useful to tell "driver didn't reach (丢步/堵转)" apart from "mechanism slipped (机械)". */
extern volatile int32_t turntable_debug_fwd_cpos;

/* 位置误差(目标-实时),由 turntable_read_perr() 刷新,调试器里看。 */
extern volatile int32_t turntable_debug_perr;

/* 状态标志字节,由 turntable_read_flag() 刷新。位含义见 turntable_read_flag()。 */
extern volatile uint8_t  turntable_debug_flag;

/* 状态标志位掩码 (X42S 手册 5.5.15) */
#define TURNTABLE_FLAG_ENABLE     0x01U  /* 使能 Ens_TF */
#define TURNTABLE_FLAG_ARRIVED    0x02U  /* 位置到达 Prf_TF */
#define TURNTABLE_FLAG_CLOG       0x04U  /* 堵转标志 Cgi_TF */
#define TURNTABLE_FLAG_CLOG_PROT  0x08U  /* 堵转保护标志 Cgp_TF */

#endif /* __TURNTABLE_APP_H */
