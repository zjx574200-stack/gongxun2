/**
 ******************************************************************************
 * zdt_motor_example.c — 使用示例 & 模板
 *
 * ✏️ 这个文件就是你要写控制逻辑的地方
 * 🔒 zdt_motor.h / zdt_motor.c / zdt_motor_port.c 都不需要动
 *
 * 本文件结构:
 *   第1部分: 🔒 固定的初始化和中断 (复制粘贴即可)
 *   第2部分: ✏️ 你的控制逻辑 (按需求写)
 *   第3部分: ✏️ 每种功能的写法速查
 ******************************************************************************
 */

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "zdt_motor.h"
#include "zdt_motor_port.h"

/* ============================================================
 * 第1部分: 🔒 固定模板 — 声明电机 + 中断 + 初始化
 * 复制粘贴到你的 main.c, 不用改
 * ============================================================ */

/* ---- 声明电机实例 ---- */
/* 传送带总线(USART3) 2个 + 底盘总线(UART4) 1个 */
zdt_t conveyor1;   /* 传送带1 (USART3, ID 1) */
zdt_t conveyor2;   /* 传送带2 (USART3, ID 2) */
zdt_t chassis;     /* 底盘   (UART4, ID 1) */

/* ---- 等待电机应答: 用移植层自带的 zdt_port_wait_rx() ---- */
/* 用法: zdt_read(&chassis, ZDT_R_CPOS);
 *       if(zdt_port_wait_rx(ZDT_BUS_CHASSIS, 100)) { ... } */

/* ---- 串口中断处理 (放在 stm32fxxx_it.c 的 USARTx_IRQHandler 里) ---- */
/*
 * 🔒 把下面这段复制到 stm32fxxx_it.c 的对应中断函数中:
 *
 * void USART1_IRQHandler(void)         // ← 改成你的串口号
 * {
 *     // ↓↓↓ 复制下面4行 ↓↓↓
 *     if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {  // ← 改成你的串口句柄
 *         __HAL_UART_CLEAR_IDLEFLAG(&huart1);              // ← 同上
 *         zdt_port_on_idle(&huart1);                       // ← 同上
 *     }
 *     // ↑↑↑ 复制上面4行 ↑↑↑
 *     HAL_UART_IRQHandler(&huart1);     // ← 这行CubeMX已生成, 保留
 * }
 */

/* ============================================================
 * 第2部分: ✏️ 你的 main 函数 — 在这里写控制逻辑
 * ============================================================ */

int main(void)
{
    /* ---- 🔒 固定初始化 (CubeMX 生成 + 电机库) ---- */
    HAL_Init();
    SystemClock_Config();        /* ✏️ CubeMX生成的时钟配置 */
    MX_GPIO_Init();              /* ✏️ CubeMX生成 */
    MX_DMA_Init();               /* ✏️ CubeMX生成 */
    MX_USART1_UART_Init();       /* ✏️ CubeMX生成, 改成你的串口初始化函数名 */

    zdt_port_init();             /* 🔒 启动UART DMA接收, 不用改 */

    /* ---- ✏️ 初始化电机实例 ---- */
    /*
     * 参数1: &实例名
     * 参数2: 固件类型
     *        ZDT_FW_EMM → Emm固件(默认), 位置用脉冲数, 速度直接RPM
     *        ZDT_FW_X   → X固件, 位置×0.1°, 速度×0.1RPM
     * 参数3: 电机ID地址 (默认=1, 如果改过ID就填改后的值)
     * 参数4: 发送回调, 固定填 zdt_port_send
     */

    /* 示例: 3台电机, 都是Emm固件, ID分别为1,2,3 */
    zdt_init(&conveyor1, ZDT_FW_EMM, 1, zdt_port3_send);   /* 传送带1 */
    zdt_init(&conveyor2, ZDT_FW_EMM, 2, zdt_port3_send);   /* 传送带2 */
    zdt_init(&chassis,   ZDT_FW_EMM, 1, zdt_port4_send);   /* 底盘 */

    /*
     * ✏️ 如果你用的是 X固件, 这样初始化:
     * zdt_t motor_x;
     * zdt_init(&motor_x, ZDT_FW_X, 1, zdt_port_send);
     */

    /* 🔒 等驱动器上电完成 (必须!) */
    HAL_Delay(500);

    /* ============================================================
     * ✏️ 下面开始写你的控制逻辑
     *
     * 每种功能的写法见 第3部分: 功能速查
     * 把不需要的删掉, 保留你需要的即可
     * ============================================================ */

    /* ---- 基本操作: 使能 → 运动 → 停止 ---- */
    zdt_enable(&conveyor1, true);     /* 使能传送带1 */
    zdt_enable(&conveyor2, true);     /* 使能传送带2 */
    zdt_enable(&chassis, true);       /* 使能底盘 */
    HAL_Delay(10);

    /* 传送带: 速度模式持续转 (600 RPM) */
    zdt_run_vel(&conveyor1, ZDT_DIR_CW, 10, 600);
    zdt_run_vel(&conveyor2, ZDT_DIR_CW, 10, 600);

    /* 底盘: 位置模式转到 90° (寻址分拣) */
    zdt_move_to(&chassis, ZDT_DIR_CW, 500, 90.0, ZDT_REF_ABS);
    HAL_Delay(1000);

    /* ---- 读状态 (底盘位置) ---- */
    zdt_read(&chassis, ZDT_R_CPOS);   /* 发查询: 实时位置 */
    if (zdt_port_wait_rx(ZDT_BUS_CHASSIS, 100)) {
        /* 数据在 zdt_port_rx_data(ZDT_BUS_CHASSIS) 指向的缓冲区里
         * buf[0]=地址, buf[1]=参数码, buf[2..]=数据, buf[最后]=0x6B
         * 具体解析见下面的 "读参数返回值解析" */
    }

    /* ---- 主循环 ---- */
    while (1) {
        /*
         * ✏️ 在这里写:
         *   - 按键检测 → 控制电机
         *   - 状态机
         *   - 定时读取位置/速度
         *   - 错误处理
         */
    }
}

/* ============================================================
 * 第3部分: ✏️ 每种功能的写法速查
 * 按 Ctrl+F 搜索关键词跳到你需要的功能
 * ============================================================ */

/* =================================================================
 * 📌 功能: 使能 / 关闭电机
 * ================================================================= */
/*
 * zdt_enable(&motor1, true);    // 使能 (电机上电, 可以转)
 * zdt_enable(&motor1, false);   // 关闭 (电机断电, 用手能拧动)
 */

/* =================================================================
 * 📌 功能: 位置模式 — 让电机转到指定角度 (最常用!)
 * ================================================================= */
/*
 * // 最简单: 逆时针 1000RPM 转 360° (1圈)
 * zdt_move_to(&motor1, ZDT_DIR_CCW, 1000, 360.0, ZDT_REF_ABS);
 *
 * // 顺时针 500RPM 转 90° (1/4圈)
 * zdt_move_to(&motor1, ZDT_DIR_CW, 500, 90.0, ZDT_REF_ABS);
 *
 * // 相对当前位置再转 180° (半圈)
 * zdt_move_to(&motor1, ZDT_DIR_CCW, 800, 180.0, ZDT_REF_REL);
 *
 * // 回到原点 (假设已回零, 原点=0°)
 * zdt_move_to(&motor1, ZDT_DIR_CCW, 1000, 0, ZDT_REF_ZERO);
 *
 * ✏️ 参数说明:
 *   第2个: ZDT_DIR_CW(顺时针) 或 ZDT_DIR_CCW(逆时针)
 *   第3个: 速度, 单位 RPM, 范围 0~3000 (Emm固件) 或 0~3000 (X固件)
 *   第4个: 目标角度, 单位"度", 360.0=1圈, 7200.0=20圈
 *   第5个: ZDT_REF_ABS(连续多段) / ZDT_REF_ZERO(回原点) / ZDT_REF_REL(相对)
 */

/* =================================================================
 * 📌 功能: 梯形轨迹 — 带加减速的平滑运动
 * ================================================================= */
/*
 * zdt_move_traj(&motor1,
 *     ZDT_DIR_CW,     // 方向
 *     500,             // 加速度 (Emm: 0-255系数, X: RPM/s)
 *     300,             // 减速度 (Emm: 无效, X: RPM/s)
 *     1500,            // 最大速度 RPM
 *     3600.0,          // 转10圈
 *     ZDT_REF_ABS);
 *
 * ✏️ 比 zdt_move_to 多了 acc/dec 两个参数:
 *   Emm固件: acc 填 0-255, 数字越大加速越快, 0=最快
 *   X固件:   acc/dec 填 RPM/s, 如 500 表示每秒加速500RPM
 */

/* =================================================================
 * 📌 功能: 速度模式 — 持续旋转不停
 * ================================================================= */
/*
 * // 以 500RPM 逆时针持续转
 * zdt_run_vel(&motor1, ZDT_DIR_CCW, 10, 500);
 * HAL_Delay(5000);       // 转5秒
 * zdt_stop(&motor1);     // 停!
 *
 * // 1000RPM 顺时针
 * zdt_run_vel(&motor1, ZDT_DIR_CW, 20, 1000);
 *
 * ✏️ 参数:
 *   第3个 acc: Emm固件 0-255, X固件 RPM/s
 *   第4个 vel: 目标速度 RPM
 */

/* =================================================================
 * 📌 功能: 回零 — 找到机械零点
 * ================================================================= */
/*
 * // 堵转碰撞回零 (最常用! 电机轴碰到硬限位后自动停止并设为零点)
 * zdt_origin_go(&motor1, ZDT_ORG_SL_BLOCK);
 * HAL_Delay(10000);  // 等回零完成, 时间因行程而异
 *
 * // 近点回零 (需接原点传感器, 要用 zdt_motor_port.h 的IO)
 * zdt_origin_go(&motor1, ZDT_ORG_NEAR);
 *
 * // 强制中断回零
 * zdt_origin_stop(&motor1);
 *
 * // 设当前位置为零点 (不运动, 软件清零)
 * zdt_origin_set_here(&motor1, true);  // true=永久保存, false=临时
 */

/* =================================================================
 * 📌 功能: 读参数 — 查询电机状态
 * ================================================================= */
/*
 * // 读实时位置
 * zdt_read(&motor1, ZDT_R_CPOS);
 * if (wait_rx(100)) {
 *     uint8_t *buf = zdt_port_rx_data();
 *     // 返回格式: [addr(1)] [param(1)] [data(N)] [0x6B(1)]
 *     // Emm固件: 位置 = (int32)(buf[2]<<24|buf[3]<<16|buf[4]<<8|buf[5]) 脉冲
 *     // X固件:   位置 = (int32)(...) / 10.0  度
 * }
 *
 * // 其他可读参数:
 * zdt_read(&motor1, ZDT_R_VEL);    // 转速
 * zdt_read(&motor1, ZDT_R_TEMP);   // 温度
 * zdt_read(&motor1, ZDT_R_CBUS);   // 总线电流
 * zdt_read(&motor1, ZDT_R_VBUS);   // 总线电压
 * zdt_read(&motor1, ZDT_R_ENCO);   // 编码器原始值
 * zdt_read(&motor1, ZDT_R_ENCL);   // 编码器校准值
 * zdt_read(&motor1, ZDT_R_OFLAG);  // 原点状态标志 (回零是否完成)
 * zdt_read(&motor1, ZDT_R_FLAG);   // 电机状态标志 (堵转/过温/过流)
 * zdt_read(&motor1, ZDT_R_PERR);   // 位置误差 (闭环跟踪偏差)
 * zdt_read(&motor1, ZDT_R_SYS);    // 系统状态
 */

/* =================================================================
 * 📌 读参数返回值解析 (重要!)
 * ================================================================= */
/*
 * zdt_read() 发送查询命令后, 驱动器会通过UART返回数据。
 * 返回帧格式:
 *   Emm固件: [addr][参数码][data_N_bytes...][0x6B]
 *   X固件:   同上
 *
 * 例: 读实时位置返回4字节:
 *   buf[0] = 电机地址
 *   buf[1] = 0x36 (ZDT_R_CPOS 的参数码)
 *   buf[2]~buf[5] = int32_t 位置值 (大端序)
 *   buf[6] = 0x6B (校验)
 *
 * Emm固件转角度: 角度 = 脉冲值 * 360.0 / 每圈脉冲数
 *   默认16细分: 角度 = 脉冲值 / 3200.0 * 360.0 = 脉冲值 * 0.1125
 *
 * X固件转角度: 角度 = 原始值 / 10.0
 *
 * 例: 读转速返回2字节:
 *   buf[2]~buf[3] = int16_t 转速值 (大端序)
 *   Emm固件: 直接就是RPM
 *   X固件:   / 10.0 才是RPM
 */

/* =================================================================
 * 📌 功能: 修改配置
 * ================================================================= */
/*
 * // 改闭环电流为 2500mA (永久保存到驱动器Flash)
 * zdt_set_foc_ma(&motor1, true, 2500);
 * HAL_Delay(100);
 *
 * // 改微步细分为 32 (默认16)
 * zdt_set_microstep(&motor1, true, 32);
 * // ⚠️ 改了细分后, Emm固件必须同步更新每圈脉冲数!
 * zdt_set_pulses_per_rev(&motor1, 32 * 200);  // 32细分 = 6400脉冲/圈
 * HAL_Delay(100);
 *
 * // 改旋转方向 (永久)
 * zdt_set_dir(&motor1, true, ZDT_DIR_CCW);
 *
 * // 切换到闭环FOC模式
 * zdt_set_ctrl_mode(&motor1, true, true);   // true=闭环, false=开环
 *
 * // 改电机ID为5 ⚠️ 改完后原ID失效, 必须用新ID通信
 * // zdt_set_id(&motor1, true, 5);
 *
 * // 改位置到达窗口为 0.5° (值=5, 因为单位是0.1°)
 * zdt_set_pos_window(&motor1, true, 5);
 *
 * // 改PID参数 (一般不需要动)
 * zdt_set_pid(&motor1, true,
 *     126640,   // 位置环Kp(梯形)
 *     126640,   // 位置环Kp(直通)
 *     15600,    // 速度环Kp
 *     26);      // 速度环Ki
 *
 * // 恢复出厂设置 (出问题时用)
 * zdt_restore_factory(&motor1);
 */

/* =================================================================
 * 📌 功能: 编码器校准
 * ================================================================= */
/*
 * // 首次使用、更换电机后应执行一次
 * // 电机会微动几下, 完成后自动保存
 * zdt_encoder_cal(&motor1);
 * HAL_Delay(3000);  // 等待校准完成
 */

/* =================================================================
 * 📌 功能: 力矩模式 — 控制输出力矩而非位置
 * ================================================================= */
/*
 * // 以 100mA/s 的斜率, 输出 500mA 力矩, CW方向
 * zdt_run_torque(&motor1, ZDT_DIR_CW, 100, 500);
 *
 * // 力矩模式 + 限速 1000RPM
 * zdt_run_torque_lv(&motor1, ZDT_DIR_CCW, 200, 800, 1000);
 */

/* =================================================================
 * 📌 功能: 快速轨迹位置 — 连续发多个位置, 效率更高
 *         适合: 3D打印、CNC、频繁换目标位置的场景
 * ================================================================= */
/*
 * // 先设置运动参数 (一次)
 * zdt_qt_set_params(&motor1, 10, 0, 800, ZDT_REF_ABS);
 *
 * // 然后连续发位置 (每次只发目标位置, 帧更短)
 * zdt_qt_move(&motor1, 90.0);    // 转到90°
 * HAL_Delay(500);
 * zdt_qt_move(&motor1, 180.0);   // 转到180°
 * HAL_Delay(800);
 * zdt_qt_move(&motor1, 360.0);   // 转到360°
 * HAL_Delay(1500);
 */

/* =================================================================
 * 📌 功能: 多电机控制
 * ================================================================= */
/*
 * // 各自独立运动
 * zdt_enable(&motor1, true);
 * zdt_enable(&motor2, true);
 * HAL_Delay(10);
 *
 * zdt_move_to(&motor1, ZDT_DIR_CCW, 1000, 360.0, ZDT_REF_ABS);
 * HAL_Delay(5);  // 帧之间稍等一下
 * zdt_move_to(&motor2, ZDT_DIR_CW, 500, 180.0, ZDT_REF_ABS);
 *
 * // 多电机同步 (需要用到 sync 标志, 见 README 协议详解)
 */

/* =================================================================
 * 📌 常用参数速查表
 * ================================================================= */
/*
 * 速度范围:    0 ~ 3000 RPM
 * 电流范围:    0 ~ 6000 mA (X42S建议 ≤4000)
 * 位置范围:    0 ~ 429496729.5° (X固件) 约1.2亿圈
 * 微步范围:    1 ~ 255, 0=256
 * 默认ID:      1
 * 默认细分:    16 (每圈3200脉冲)
 * 默认电流:    ~2000mA
 * 位置窗口:    默认8 (即0.8°)
 */

/* ============================================================
 * 🔒 下面的函数是 CubeMX 生成的模板, 不用动
 * ============================================================ */

void SystemClock_Config(void)
{
    /* ✏️ 这里是你 CubeMX 生成的时钟配置, 不用动 */
    RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState = RCC_HSE_ON;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 4; o.PLL.PLLN = 168; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&o);
    c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1; c.APB1CLKDivider = RCC_HCLK_DIV4; c.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5);
}

void Error_Handler(void) { while(1); }
