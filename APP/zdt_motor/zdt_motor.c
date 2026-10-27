/**
 ******************************************************************************
 * @file    zdt_motor.c
 * @brief   ZDT_XS 闭环步进电机驱动库 (串口版)
 *
 * 协议: [addr][cmd1][cmd2][data...][checksum 0x6B]
 * 传输: UART DMA, 波特率由驱动器配置决定 (默认115200)
 *
 * X固件 vs Emm固件:
 *   - X:  位置×10(0.1°), 速度×10(0.1RPM), 加速度 RPM/s
 *   - Emm: 位置=脉冲数(int32), 速度=RPM(uint16), 加速度=0-255系数
 *
 * 基于: X_V2 (CAN协议) + Emm_V5 (串口协议)
 ******************************************************************************
 */

#include "zdt_motor.h"

/* ============================================================
 *  内部辅助
 * ============================================================ */

#define ABS(x)  ((x) > 0 ? (x) : -(x))

/* 给电机实例发送一条命令 */
static void zdt_send(zdt_t *m, uint8_t len)
{
    if (m->send) m->send(m->tx, len);
}

/* 初始化命令缓冲: [addr][cmd1][cmd2] */
static void zdt_cmd(zdt_t *m, uint8_t cmd1, uint8_t cmd2)
{
    m->tx[0] = m->addr;
    m->tx[1] = cmd1;
    m->tx[2] = cmd2;
}

/* 追加 uint8 */
static void zdt_u8(zdt_t *m, uint8_t idx, uint8_t v)
{
    m->tx[idx] = v;
}

/* 追加 uint16 (大端) */
static void zdt_u16(zdt_t *m, uint8_t idx, uint16_t v)
{
    m->tx[idx]     = (uint8_t)(v >> 8);
    m->tx[idx + 1] = (uint8_t)(v);
}

/* 追加 uint32 (大端) */
static void zdt_u32(zdt_t *m, uint8_t idx, uint32_t v)
{
    m->tx[idx]     = (uint8_t)(v >> 24);
    m->tx[idx + 1] = (uint8_t)(v >> 16);
    m->tx[idx + 2] = (uint8_t)(v >> 8);
    m->tx[idx + 3] = (uint8_t)(v);
}

/* 追加 int32 (大端) */
static void zdt_i32(zdt_t *m, uint8_t idx, int32_t v)
{
    zdt_u32(m, idx, (uint32_t)v);
}

/* 尾部追加 checksum 0x6B */
static void zdt_end(zdt_t *m, uint8_t idx)
{
    m->tx[idx] = 0x6B;
}

/* 度 → 脉冲 (Emm固件) */
static int32_t deg_to_pulse(zdt_t *m, float deg)
{
    return (int32_t)(deg * m->pulses_per_rev / 360.0f);
}

/* ============================================================
 *  初始化
 * ============================================================ */

void zdt_init(zdt_t *m, zdt_fw_t fw, uint8_t addr,
              void (*send_fn)(uint8_t *buf, uint8_t len))
{
    memset(m, 0, sizeof(zdt_t));
    m->fw   = fw;
    m->addr = addr;
    m->send = send_fn;
    m->pulses_per_rev = 3200;  /* 默认16细分 */
}

void zdt_set_pulses_per_rev(zdt_t *m, uint16_t ppr)
{
    m->pulses_per_rev = ppr;
}

/* ============================================================
 *  校准 & 复位
 *  格式: [addr][cmd1][cmd2][0x6B]  长度4
 * ============================================================ */

void zdt_encoder_cal(zdt_t *m)
{
    zdt_cmd(m, 0x06, 0x45); zdt_end(m, 3); zdt_send(m, 4);
}
void zdt_reset(zdt_t *m)
{
    zdt_cmd(m, 0x08, 0x97); zdt_end(m, 3); zdt_send(m, 4);
}
void zdt_zero_pos(zdt_t *m)
{
    zdt_cmd(m, 0x0A, 0x6D); zdt_end(m, 3); zdt_send(m, 4);
}
void zdt_reset_clog(zdt_t *m)
{
    zdt_cmd(m, 0x0E, 0x52); zdt_end(m, 3); zdt_send(m, 4);
}
void zdt_restore_factory(zdt_t *m)
{
    zdt_cmd(m, 0x0F, 0x5F); zdt_end(m, 3); zdt_send(m, 4);
}

/* ============================================================
 *  使能
 *  格式: [addr][0xF3][0xAB][state][snF][0x6B]  长度6
 * ============================================================ */

void zdt_enable(zdt_t *m, bool on)
{
    zdt_cmd(m, 0xF3, 0xAB);
    zdt_u8(m, 3, (uint8_t)on);   /* state */
    zdt_u8(m, 4, 0);             /* snF = 单独执行 */
    zdt_end(m, 5);
    zdt_send(m, 6);
}

/* ============================================================
 *  速度模式
 *
 *  X固件: [addr][0xF6][dir][vel_h][vel_l][acc_h][acc_l][snF][0x6B]  长度9
 *          vel = RPM × 10
 *
 *  Emm固件: [addr][0xF6][dir][vel_h][vel_l][acc][snF][0x6B]  长度8
 *           vel = RPM, acc = 0-255
 * ============================================================ */

void zdt_run_vel(zdt_t *m, zdt_dir_t dir, uint16_t acc, float vel_rpm)
{
    zdt_cmd(m, 0xF6, (uint8_t)dir);

    if (m->fw == ZDT_FW_EMM) {
        zdt_u16(m, 3, (uint16_t)vel_rpm);    /* RPM 整数 */
        zdt_u8(m, 5, (uint8_t)(acc > 255 ? 255 : acc));
        zdt_u8(m, 6, 0);                      /* snF */
        zdt_end(m, 7);
        zdt_send(m, 8);
    } else {
        zdt_u16(m, 3, (uint16_t)ABS(acc));           /* acc RPM/s */
        zdt_u16(m, 5, (uint16_t)(vel_rpm * 10.0f));  /* RPM ×10 */
        zdt_u8(m, 7, 0);                             /* snF */
        zdt_end(m, 8);
        zdt_send(m, 9);
    }
}

/* ============================================================
 *  直通位置模式
 *
 *  X固件: [addr][0xFB][dir][vel_h][vel_l][pos24..0][raf][snF][0x6B]  长度12
 *          vel = RPM × 10, pos = 度 × 10 (uint32)
 *
 *  Emm固件: [addr][0xFD][dir][vel_h][vel_l][acc][clk24..0][raf][snF][0x6B]  长度13
 *           vel = RPM, acc = 0-255, clk = 脉冲数(uint32)
 * ============================================================ */

void zdt_move_to(zdt_t *m, zdt_dir_t dir, float vel_rpm, float pos_deg, zdt_ref_t ref)
{
    if (m->fw == ZDT_FW_EMM) {
        zdt_cmd(m, 0xFD, (uint8_t)dir);
        zdt_u16(m, 3, (uint16_t)vel_rpm);          /* RPM */
        zdt_u8(m, 5, 0);                            /* acc=0(最快) */
        zdt_u32(m, 6, (uint32_t)deg_to_pulse(m, pos_deg)); /* 脉冲 */
        zdt_u8(m, 10, (uint8_t)ref);
        zdt_u8(m, 11, 0);                           /* snF */
        zdt_end(m, 12);
        zdt_send(m, 13);
    } else {
        zdt_cmd(m, 0xFB, (uint8_t)dir);
        zdt_u16(m, 3, (uint16_t)(vel_rpm * 10.0f));       /* ×10 */
        zdt_u32(m, 5, (uint32_t)(pos_deg * 10.0f));       /* ×10 */
        zdt_u8(m, 9, (uint8_t)ref);
        zdt_u8(m, 10, 0);                                  /* snF */
        zdt_end(m, 11);
        zdt_send(m, 12);
    }
}

/* ============================================================
 *  梯形轨迹位置模式
 *
 *  X固件: [addr][0xFD][dir][acc_h][acc_l][dec_h][dec_l][vel_h][vel_l]
 *          [pos24..0][raf][snF][0x6B]  长度16
 *
 *  Emm固件: 与直通相同(Emm无独立梯形加减速,acc参数控制加速度)
 * ============================================================ */

void zdt_move_traj(zdt_t *m, zdt_dir_t dir,
                   uint16_t acc, uint16_t dec,
                   float vel_rpm, float pos_deg, zdt_ref_t ref)
{
    if (m->fw == ZDT_FW_EMM) {
        /* Emm: 用 acc 系数控制加速度 */
        zdt_cmd(m, 0xFD, (uint8_t)dir);
        zdt_u16(m, 3, (uint16_t)vel_rpm);
        zdt_u8(m, 5, (uint8_t)(acc > 255 ? 255 : acc));
        zdt_u32(m, 6, (uint32_t)deg_to_pulse(m, pos_deg));
        zdt_u8(m, 10, (uint8_t)ref);
        zdt_u8(m, 11, 0);
        zdt_end(m, 12);
        zdt_send(m, 13);
    } else {
        zdt_cmd(m, 0xFD, (uint8_t)dir);
        zdt_u16(m, 3, acc);                               /* 加速度 RPM/s */
        zdt_u16(m, 5, dec);                               /* 减速度 RPM/s */
        zdt_u16(m, 7, (uint16_t)(vel_rpm * 10.0f));       /* ×10 */
        zdt_u32(m, 9, (uint32_t)(pos_deg * 10.0f));       /* ×10 */
        zdt_u8(m, 13, (uint8_t)ref);
        zdt_u8(m, 14, 0);                                 /* snF */
        zdt_end(m, 15);
        zdt_send(m, 16);
    }
}

/* ============================================================
 *  力矩模式
 *
 *  X固件: [addr][0xF5][sign][ramp_h][ramp_l][torque_h][torque_l][snF][0x6B]  长度9
 *         ramp: mA/s, torque: mA
 *
 *  力矩+限速 X: [addr][0xC5][sign][ramp_h][ramp_l][torque_h][torque_l][snF][vel_h][vel_l][0x6B]  长度11
 * ============================================================ */

void zdt_run_torque(zdt_t *m, zdt_dir_t sign,
                    uint16_t ramp_ma_s, uint16_t torque_ma)
{
    zdt_cmd(m, 0xF5, (uint8_t)sign);
    zdt_u16(m, 3, ramp_ma_s);
    zdt_u16(m, 5, torque_ma);
    zdt_u8(m, 7, 0);          /* snF */
    zdt_end(m, 8);
    zdt_send(m, 9);
}

void zdt_run_torque_lv(zdt_t *m, zdt_dir_t sign,
                       uint16_t ramp_ma_s, uint16_t torque_ma, float max_vel_rpm)
{
    zdt_cmd(m, 0xC5, (uint8_t)sign);
    zdt_u16(m, 3, ramp_ma_s);
    zdt_u16(m, 5, torque_ma);
    zdt_u8(m, 7, 0);          /* snF */
    zdt_u16(m, 8, (uint16_t)(max_vel_rpm * 10.0f));
    zdt_end(m, 10);
    zdt_send(m, 11);
}

/* ============================================================
 *  急停 & 同步
 * ============================================================ */

void zdt_stop(zdt_t *m)
{
    zdt_cmd(m, 0xFE, 0x98);
    zdt_u8(m, 3, 0);          /* snF = 单独 */
    zdt_end(m, 4);
    zdt_send(m, 5);
}

void zdt_sync_go(zdt_t *m)
{
    zdt_cmd(m, 0xFF, 0x66);
    zdt_end(m, 3);
    zdt_send(m, 4);
}

/* ============================================================
 *  快速轨迹位置
 *
 *  设参数 X: [addr][0xF1][acc_h][acc_l][dec_h][dec_l][vel_h][vel_l][raf][snF][max_h][max_l][0x6B]  长度13
 *  设参数 Emm: [addr][0xF1][vel_h][vel_l][acc][raf][snF][0x6B]  长度8
 *  发位置 X: [addr][0xFC][pos24..0][0x6B]  长度7
 *  发位置 Emm: [addr][0xFC][clk24..0][0x6B]  长度7
 * ============================================================ */

void zdt_qt_set_params(zdt_t *m, uint16_t acc, uint16_t dec,
                        float vel_rpm, zdt_ref_t ref)
{
    if (m->fw == ZDT_FW_EMM) {
        zdt_cmd(m, 0xF1, 0x00);   /* Emm QPos 不需要cmd2 */
        zdt_u16(m, 2, (uint16_t)vel_rpm);
        zdt_u8(m, 4, (uint8_t)(acc > 255 ? 255 : acc));
        zdt_u8(m, 5, (uint8_t)ref);
        zdt_u8(m, 6, 0);          /* snF */
        zdt_end(m, 7);
        zdt_send(m, 8);
    } else {
        zdt_cmd(m, 0xF1, 0x00);
        zdt_u16(m, 2, acc);
        zdt_u16(m, 4, dec);
        zdt_u16(m, 6, (uint16_t)(vel_rpm * 10.0f));
        zdt_u8(m, 8, (uint8_t)ref);
        zdt_u8(m, 9, 0);          /* snF */
        zdt_u16(m, 10, 0);        /* maxCurrent = 默认 */
        zdt_end(m, 12);
        zdt_send(m, 13);
    }
}

void zdt_qt_move(zdt_t *m, float pos_deg)
{
    zdt_cmd(m, 0xFC, 0x00);
    if (m->fw == ZDT_FW_EMM) {
        zdt_i32(m, 2, deg_to_pulse(m, pos_deg));
    } else {
        zdt_i32(m, 2, (int32_t)(pos_deg * 10.0f));
    }
    zdt_end(m, 6);
    zdt_send(m, 7);
}

/* ============================================================
 *  回零
 * ============================================================ */

void zdt_origin_set_here(zdt_t *m, bool save)
{
    zdt_cmd(m, 0x93, 0x88);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_end(m, 4);
    zdt_send(m, 5);
}

void zdt_origin_go(zdt_t *m, zdt_origin_mode_t mode)
{
    zdt_cmd(m, 0x9A, (uint8_t)mode);
    zdt_u8(m, 3, 0);          /* snF = 单独 */
    zdt_end(m, 4);
    zdt_send(m, 5);
}

void zdt_origin_stop(zdt_t *m)
{
    zdt_cmd(m, 0x9C, 0x48);
    zdt_end(m, 3);
    zdt_send(m, 4);
}

/* ============================================================
 *  读参数
 *  格式: [addr][cmd(即参数对应的命令字节)][0x6B]  长度3
 * ============================================================ */

void zdt_read(zdt_t *m, zdt_read_t param)
{
    zdt_cmd(m, (uint8_t)param, 0x00);  /* cmd1=参数类型码, cmd2不需要 */
    m->tx[2] = 0x6B;                    /* 只有3字节 */
    zdt_send(m, 3);
}

/* Emm 固件用: 命令字节直接给, 不做枚举映射 */
void zdt_read_raw(zdt_t *m, uint8_t cmd)
{
    m->tx[0] = m->addr;
    m->tx[1] = cmd;
    m->tx[2] = 0x6B;
    zdt_send(m, 3);
}

/* ============================================================
 *  写配置
 * ============================================================ */

void zdt_set_id(zdt_t *m, bool save, uint8_t new_id)
{
    zdt_cmd(m, 0xAE, 0x4B);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, new_id);
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_microstep(zdt_t *m, bool save, uint8_t ms)
{
    zdt_cmd(m, 0x84, 0x8A);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, ms);
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_dir(zdt_t *m, bool save, zdt_dir_t dir)
{
    zdt_cmd(m, 0xD4, 0x60);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, (uint8_t)dir);
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_type(zdt_t *m, bool save, bool is_0_9deg)
{
    zdt_cmd(m, 0xD7, 0x35);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, (uint8_t)(is_0_9deg ? 25 : 50));
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_ctrl_mode(zdt_t *m, bool save, bool is_foc)
{
    zdt_cmd(m, 0x46, 0x69);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, (uint8_t)is_foc);
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_lock_btn(zdt_t *m, bool save, bool lock)
{
    zdt_cmd(m, 0xD0, 0xB3);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u8(m, 4, (uint8_t)lock);
    zdt_end(m, 5);
    zdt_send(m, 6);
}

void zdt_set_open_ma(zdt_t *m, bool save, uint16_t ma)
{
    zdt_cmd(m, 0x44, 0x33);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u16(m, 4, ma);
    zdt_end(m, 6);
    zdt_send(m, 7);
}

void zdt_set_foc_ma(zdt_t *m, bool save, uint16_t ma)
{
    zdt_cmd(m, 0x45, 0x66);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u16(m, 4, ma);
    zdt_end(m, 6);
    zdt_send(m, 7);
}

void zdt_set_pid(zdt_t *m, bool save,
                 uint32_t pTkp, uint32_t pBkp,
                 uint32_t vkp, uint32_t vki)
{
    zdt_cmd(m, 0x4A, 0xC3);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u32(m, 4, pTkp);
    zdt_u32(m, 8, pBkp);
    zdt_u32(m, 12, vkp);
    zdt_u32(m, 16, vki);
    zdt_end(m, 20);
    zdt_send(m, 21);
}

void zdt_set_pos_window(zdt_t *m, bool save, uint16_t w_01deg)
{
    zdt_cmd(m, 0xD1, 0x07);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u16(m, 4, w_01deg);
    zdt_end(m, 6);
    zdt_send(m, 7);
}

void zdt_set_protect(zdt_t *m, bool save,
                     uint16_t otp, uint16_t ocp, uint16_t ms)
{
    zdt_cmd(m, 0xD3, 0x56);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u16(m, 4, otp);
    zdt_u16(m, 6, ocp);
    zdt_u16(m, 8, ms);
    zdt_end(m, 10);
    zdt_send(m, 11);
}

void zdt_set_heartbeat(zdt_t *m, bool save, uint32_t ms)
{
    zdt_cmd(m, 0x68, 0x38);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u32(m, 4, ms);
    zdt_end(m, 8);
    zdt_send(m, 9);
}

void zdt_set_integral(zdt_t *m, bool save, uint32_t il)
{
    zdt_cmd(m, 0x4B, 0x57);
    zdt_u8(m, 3, (uint8_t)save);
    zdt_u32(m, 4, il);
    zdt_end(m, 8);
    zdt_send(m, 9);
}
