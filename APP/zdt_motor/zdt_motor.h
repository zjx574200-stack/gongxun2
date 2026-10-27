/**
 ******************************************************************************
 * zdt_motor.h — ZDT闭环步进电机驱动库 头文件
 *
 * 🔒 本文件: 全部不用动, 直接 include 即可
 * 🔒 所有枚举、结构体、API 声明都是固定的, 不需要修改
 *
 * 你需要改的文件只有:
 *   ✏️ zdt_motor_port.h — 改 UART 句柄、引脚
 *   ✏️ zdt_motor_example.c — 写你的控制逻辑
 ******************************************************************************
 */

#ifndef __ZDT_MOTOR_H
#define __ZDT_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 🔒 枚举 — 不用动
 * ============================================================ */

/** 固件类型 (初始化时选一个) */
typedef enum {
    ZDT_FW_X    = 0,    /* X固件: 1.8°电机, 参数×10 */
    ZDT_FW_EMM  = 1,    /* Emm固件: 0.9°/1.8°电机, 参数=脉冲数 (默认) */
} zdt_fw_t;

/** 方向 */
typedef enum {
    ZDT_DIR_CW  = 0,    /* 顺时针 */
    ZDT_DIR_CCW = 1,    /* 逆时针 */
} zdt_dir_t;

/** 位置参考系:
 *  ZDT_REF_ABS  — "以当前目标为基准继续累加"     (连续多段运动用这个)
 *  ZDT_REF_ZERO — "以原点为零点, 转到绝对坐标"   (回零后用这个)
 *  ZDT_REF_REL  — "以当前实时位置为基准偏移N度"  (相对运动用这个)
 */
typedef enum {
    ZDT_REF_ABS  = 0,
    ZDT_REF_ZERO = 1,
    ZDT_REF_REL  = 2,
} zdt_ref_t;

/** 回零模式 */
typedef enum {
    ZDT_ORG_NEAR      = 0,  /* 近点+接近点回零 (需接传感器) */
    ZDT_ORG_NEAR_ONLY = 1,  /* 仅接近点回零 */
    ZDT_ORG_SL_BLOCK  = 2,  /* 堵转碰撞回零 (最常用, 无需传感器) */
    ZDT_ORG_SL_ZERO   = 3,  /* 堵转原点回零 */
} zdt_origin_mode_t;

/* ---- Emm 固件读命令字节 (配合 zdt_read_raw 使用) ----
 * 协议: 发 [addr][cmd][0x6B], 驱动器回 [addr][cmd][数据...][0x6B]
 * 命令不被识别时驱动器回 [addr][0x00][0xEE][0x6B], 一看就知道字节错了
 */
#define ZDT_EMM_RD_VBUS   0x24   /* 总线电压 */
#define ZDT_EMM_RD_CPHA   0x27   /* 相电流 */
#define ZDT_EMM_RD_ENC    0x31   /* 编码器值 */
#define ZDT_EMM_RD_VEL    0x35   /* 实时转速 */
#define ZDT_EMM_RD_POS    0x36   /* 实时位置 (已对照手册核实) */
#define ZDT_EMM_RD_PERR   0x37   /* 位置误差 */
#define ZDT_EMM_RD_FLAG   0x3A   /* 使能/到位/堵转 状态标志位 */
#define ZDT_EMM_RD_OFLAG  0x3B   /* 回零状态标志位 */

/** 可读参数
 *  ⚠️ 这套枚举是 X 系列(CAN协议)的参数索引, 只能配 ZDT_FW_X 用。
 *     Emm 固件请改用 zdt_read_raw() + 上面的 ZDT_EMM_RD_xxx 宏。
 */
typedef enum {
    ZDT_R_VBUS  = 5,    /* 总线电压 */
    ZDT_R_CBUS  = 6,    /* 总线电流 */
    ZDT_R_CPHA  = 7,    /* 相电流 */
    ZDT_R_ENCO  = 8,    /* 编码器原始值 */
    ZDT_R_CLKC  = 9,    /* 实时脉冲数 */
    ZDT_R_ENCL  = 10,   /* 编码器校准值 */
    ZDT_R_CLKI  = 11,   /* 输入脉冲数 */
    ZDT_R_TPOS  = 12,   /* 目标位置 */
    ZDT_R_SPOS  = 13,   /* 实时设定的目标位置 */
    ZDT_R_VEL   = 14,   /* 实时转速 */
    ZDT_R_CPOS  = 15,   /* 实时位置 (最常用!) */
    ZDT_R_PERR  = 16,   /* 位置误差 */
    ZDT_R_VBAT  = 17,   /* 电池电压 (Y42) */
    ZDT_R_TEMP  = 18,   /* 温度 */
    ZDT_R_FLAG  = 19,   /* 电机状态标志位 */
    ZDT_R_OFLAG = 20,   /* 原点状态标志位 */
    ZDT_R_OAF   = 21,   /* 电机+原点标志 */
    ZDT_R_PIN   = 22,   /* IO 状态 */
    ZDT_R_SYS   = 23,   /* 系统状态 */
} zdt_read_t;

/* ============================================================
 * 🔒 电机实例结构体 — 不用动
 * ============================================================ */

typedef struct {
    uint8_t   addr;             /* 电机 ID (1~247, 默认1) */
    zdt_fw_t  fw;               /* 固件类型 */
    uint16_t  pulses_per_rev;   /* Emm固件: 每圈脉冲数 (默认3200) */
    void    (*send)(uint8_t *buf, uint8_t len);  /* UART发送回调 */
    uint8_t   tx[32];           /* 内部发送缓冲 (只读) */
} zdt_t;

/* ============================================================
 * 🔒 API 声明 — 不用动, 直接调用即可
 * ============================================================ */

/** 初始化电机实例 */
void zdt_init(zdt_t *m, zdt_fw_t fw, uint8_t addr,
              void (*send_fn)(uint8_t *buf, uint8_t len));

/** 设置 Emm 固件每圈脉冲数 (改了微步细分后需要同步调用) */
void zdt_set_pulses_per_rev(zdt_t *m, uint16_t ppr);

/* ---- 校准 & 复位 ---- */
void zdt_encoder_cal(zdt_t *m);           /* 编码器校准 (首次使用或换电机后执行) */
void zdt_reset(zdt_t *m);                 /* 复位电机 */
void zdt_zero_pos(zdt_t *m);              /* 当前位置清零 */
void zdt_reset_clog(zdt_t *m);            /* 重置堵转保护 */
void zdt_restore_factory(zdt_t *m);       /* 恢复出厂设置 */

/* ---- 运动控制 ---- */
void zdt_enable(zdt_t *m, bool on);       /* 使能/关闭电机 */

/* 直通位置模式 (最常用! 无加减速, 直接到达目标) */
void zdt_move_to(zdt_t *m, zdt_dir_t dir, float vel_rpm,
                 float pos_deg, zdt_ref_t ref);

/* 梯形轨迹位置模式 (带加减速, 更平滑, 适合长距离运动) */
void zdt_move_traj(zdt_t *m, zdt_dir_t dir,
                   uint16_t acc, uint16_t dec,
                   float vel_rpm, float pos_deg, zdt_ref_t ref);

/* 速度模式 (持续旋转, 适合传送带等) */
void zdt_run_vel(zdt_t *m, zdt_dir_t dir, uint16_t acc, float vel_rpm);

/* 力矩模式 (直接控制电流/力矩) */
void zdt_run_torque(zdt_t *m, zdt_dir_t sign,
                    uint16_t ramp_ma_s, uint16_t torque_ma);
void zdt_run_torque_lv(zdt_t *m, zdt_dir_t sign,
                       uint16_t ramp_ma_s, uint16_t torque_ma, float max_vel_rpm);

void zdt_stop(zdt_t *m);                  /* 急停 */
void zdt_sync_go(zdt_t *m);               /* 同步触发 (让队列中的多电机同时动) */

/* ---- 快速轨迹位置 (先设参数, 再连续发多个位置, 适合频繁换目标) ---- */
void zdt_qt_set_params(zdt_t *m, uint16_t acc, uint16_t dec,
                        float vel_rpm, zdt_ref_t ref);
void zdt_qt_move(zdt_t *m, float pos_deg);

/* ---- 回零 ---- */
void zdt_origin_set_here(zdt_t *m, bool save);   /* 设当前位置为零点 */
void zdt_origin_go(zdt_t *m, zdt_origin_mode_t mode);  /* 开始回零 */
void zdt_origin_stop(zdt_t *m);                  /* 强制中断回零 */

/* ---- 读参数 (返回数据在串口接收缓冲里, 需要自己解析) ---- */
void zdt_read(zdt_t *m, zdt_read_t param);   /* ⚠️ 仅 X 固件, Emm 请用 zdt_read_raw */

/** 直接发一条读命令: [addr][cmd][0x6B]
 *  cmd 用 ZDT_EMM_RD_xxx 宏。应答落在移植层的接收缓冲里。 */
void zdt_read_raw(zdt_t *m, uint8_t cmd);

/* ---- 写配置 (第二个参数 save=true 则永久保存到 Flash, false=临时生效) ---- */
void zdt_set_id(zdt_t *m, bool save, uint8_t new_id);       /* 改电机ID ⚠️ */
void zdt_set_microstep(zdt_t *m, bool save, uint8_t ms);    /* 微步细分 1~255 */
void zdt_set_dir(zdt_t *m, bool save, zdt_dir_t dir);       /* 旋转方向 */
void zdt_set_type(zdt_t *m, bool save, bool is_0_9deg);    /* 电机类型 */
void zdt_set_ctrl_mode(zdt_t *m, bool save, bool is_foc);  /* 开环/闭环 */
void zdt_set_lock_btn(zdt_t *m, bool save, bool lock);      /* 锁按键 */
void zdt_set_open_ma(zdt_t *m, bool save, uint16_t ma);    /* 开环电流 mA */
void zdt_set_foc_ma(zdt_t *m, bool save, uint16_t ma);     /* 闭环电流 mA */
void zdt_set_pid(zdt_t *m, bool save, uint32_t pTkp, uint32_t pBkp,
                 uint32_t vkp, uint32_t vki);               /* PID参数 */
void zdt_set_pos_window(zdt_t *m, bool save, uint16_t w_01deg); /* 到达窗口 */
void zdt_set_protect(zdt_t *m, bool save,
                     uint16_t otp, uint16_t ocp, uint16_t ms);  /* 过温过流保护 */
void zdt_set_heartbeat(zdt_t *m, bool save, uint32_t ms);   /* 心跳保护 */
void zdt_set_integral(zdt_t *m, bool save, uint32_t il);    /* 积分限幅 */

#ifdef __cplusplus
}
#endif

#endif
