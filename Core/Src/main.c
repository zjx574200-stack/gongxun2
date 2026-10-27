/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mydefine.h"
#include "conveyor_app/conveyor_app.h"
#include "servo/servo.h"
#include "servo/servo_hw_test.h"
#include "turntable_app/turntable_app.h"
#include "vision_app/vision_app.h"
#include "oled_app/oled_app.h"
#include "screen_app/screen_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1: run only the early PB6/PB7 hardware PWM test. 0: resume existing startup. */
#define SERVO_STANDALONE_TEST 0U
/* Temporary servo test: bypass all turntable startup and positioning commands. */
#define SORTING_TURNTABLE_ENABLED 1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Watch this value to locate startup stalls without relying on source stepping. */
volatile uint32_t g_boot_stage = 0U;
  zdt_t conveyor1;   // Legacy direct handle; conveyor_app owns the active handles.
  zdt_t conveyor2;   // Conveyor IDs are configured in conveyor_app.h.
  zdt_t chassis;     // UART4 chassis handle used by the optional debug task.
  Servo_HandleTypeDef g_servo[2];   /* 舵机句柄: 通道0=PB6(270°), 通道1=PB7(180°) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============ 板载LED(PC13) + 中间红外传感�??(PG0) ============ */
/* PC13 板载LED低电平点亮�?�分拣流程用它做调试信号(不用调试器也能看结果) */
#define IR_LED_PORT        GPIOC
#define IR_LED_PIN         GPIO_PIN_13
#define IR_LED_ON          GPIO_PIN_RESET   /* PC13 低电平点�?? */
#define IR_LED_OFF         GPIO_PIN_SET

/* 物体�?测传感器: 中间传感�? PG0(F103 版是 PB13), �?测一级传送带上的物料是否经过�?
 * 实测"无物�??(对齐) = 高电�??, 有物�??(挡光) = 低电�??"(对射 NPN �??�??, 挡光拉低)�??
 * 若你的传感器反过�??(有物�?? = �??), �?? IR_OBJECT_LEVEL 改成 GPIO_PIN_SET�?? */
#define IR_SENSOR_PORT     GPIOG
#define IR_SENSOR_PIN      GPIO_PIN_0
#define IR_OBJECT_LEVEL    GPIO_PIN_RESET
#define IR_SENSOR_TIMEOUT_MS       30000U
#define IR_SENSOR_DEBOUNCE_MS         40U


/* �?级传送带"有无物料"传感�?: PG2 (F103 版是 PB5), 对着�?级传送带, �?测上面还有没有物料�??
 * 有物�? �? 新一轮舵机一不投�?, 直接跑一�?(把上�?轮没送完/手动放的料接�?�?);
 * 无物�? �? 舵机�?�? 20° 投一个新料�??
 * 电平: 有物�? = CONVEYOR1_LOAD_LEVEL(�?), 无物�? = 相反(�?)�?
 * 若实测反过来(有物�?=�?), �? CONVEYOR1_LOAD_LEVEL 改成 GPIO_PIN_SET�? */
#define CONVEYOR1_LOAD_PORT   GPIOG
#define CONVEYOR1_LOAD_PIN    GPIO_PIN_2
#define CONVEYOR1_LOAD_LEVEL  GPIO_PIN_RESET  /* low level means material present */

/* 舵机二口到位传感�?: PG1 (F103 版是 PB14), 对着舵机二投料口�?
 * 物料走到舵机二口�?, 传感器检测到 �? 二级传�?�带立即停�??
 * 电平: 有物�?(到位) = SERVO2_OBJECT_LEVEL(�?), 无物�? = 相反(�?)�?
 * 若实测反过来(有物�?=�?), �? SERVO2_OBJECT_LEVEL 改成 GPIO_PIN_SET�? */
#define SERVO2_SENSOR_PORT   GPIOG
#define SERVO2_SENSOR_PIN    GPIO_PIN_1
#define SERVO2_OBJECT_LEVEL  GPIO_PIN_RESET
#define SERVO2_SENSOR_TIMEOUT_MS 30000U
#define SERVO2_SENSOR_DEBOUNCE_MS 40U

/* Wait for the through-beam sensor at the camera/servo-2 station. */
static uint8_t servo2_wait_for_object(uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    uint32_t active_since = 0U;
    uint8_t active = 0U;

    for (;;) {
        uint32_t now = HAL_GetTick();
        GPIO_PinState state = HAL_GPIO_ReadPin(SERVO2_SENSOR_PORT, SERVO2_SENSOR_PIN);

        if (state == SERVO2_OBJECT_LEVEL) {
            if (!active) {
                active = 1U;
                active_since = now;
            } else if ((uint32_t)(now - active_since) >= SERVO2_SENSOR_DEBOUNCE_MS) {
                return 1U;
            }
        } else {
            active = 0U;
        }

        if ((uint32_t)(now - started) >= timeout_ms) {
            return 0U;
        }
        HAL_Delay(1U);
    }
}

static void led_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin   = IR_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_LED_PORT, &gpio);
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);
}

/* �?级传送带"有无物料"传感�?(PG2): 输入 + 上拉(不接时也稳定)�?
 * 覆盖 CubeMX �? seensor3(PG2) �? NOPULL 配置, �? F103 �? PB5 的做法一致�?? */
static void conveyor1_load_sensor_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Pin  = CONVEYOR1_LOAD_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CONVEYOR1_LOAD_PORT, &gpio);
}

/* Require the PG0 active level to remain stable before handing the part to
 * conveyor 2.  This prevents a power-up edge or sensor noise from stopping
 * conveyor 1 immediately after it starts. */
static uint8_t ir_wait_for_object(uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    uint32_t active_since = 0U;
    uint8_t active = 0U;

    for (;;) {
        uint32_t now = HAL_GetTick();
        GPIO_PinState state = HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN);

        if (state == IR_OBJECT_LEVEL) {
            if (!active) {
                active = 1U;
                active_since = now;
            } else if ((uint32_t)(now - active_since) >= IR_SENSOR_DEBOUNCE_MS) {
                return 1U;
            }
        } else {
            active = 0U;
        }

        if ((uint32_t)(now - started) >= timeout_ms) {
            return 0U;
        }
        HAL_Delay(1U);
    }
}

/* 第一舵机(通道0=PB6, 270°舵机) 的复位位 = 上电时的"当前位置"�?? */
static float servo1_home_deg = 95.0f;

static void servo1_open_90(void)
{
    Servo_MoveTo(&g_servo[0], servo1_home_deg - 20.0f, 1000);
    while (Servo_IsMoving(&g_servo[0])) { Servo_Update(&g_servo[0]); HAL_Delay(1); }
}

static void servo1_reset(void)
{
    Servo_MoveTo(&g_servo[0], servo1_home_deg, 1000);
    while (Servo_IsMoving(&g_servo[0])) { Servo_Update(&g_servo[0]); HAL_Delay(1); }
}

/* 第二舵机(通道1=PB7, 180°舵机) 的复位位 = 上电时的"当前位置"�?? */
static float servo2_home_deg = 150.0f;

static void servo2_open_90(void)
{
    Servo_MoveTo(&g_servo[1], servo2_home_deg - 80.0f, 1000);
    while (Servo_IsMoving(&g_servo[1])) { Servo_Update(&g_servo[1]); HAL_Delay(1); }
}

static void servo2_reset(void)
{
    Servo_MoveTo(&g_servo[1], servo2_home_deg, 1000);
    while (Servo_IsMoving(&g_servo[1])) { Servo_Update(&g_servo[1]); HAL_Delay(1); }
}

/* PC13 �?? n �??: 每次�?? on_ms、灭 off_ms。做调试信号�?? */
static void led_blink_times(uint8_t n, uint16_t on_ms, uint16_t off_ms)
{
    while (n--) {
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);
        HAL_Delay(on_ms);
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);
        HAL_Delay(off_ms);
    }
}

/* ============ OLED 双摄像头识别结果显示 ============ */

/* �? OLED 上显示两台摄像头的原始结果�?�融合结果和命中的融合规则�??
 * 目标�? OLED 只有四行，因此由 oled_show_cameras() 压缩�? LS/YB/F/R 四行�? */
static void oled_show_result(void)
{
    /* The source F103 OLED shows the two raw camera results, the fused
     * result, and the rule that selected it.  Keep that semantic layout on
     * the target's four-line 8x16 OLED. */
    oled_show_cameras((int32_t)g_vision_shape0,
                      (int32_t)g_vision_shape1,
                      (int32_t)g_vision_color1,
                      (int32_t)g_vision_shape,
                      (int32_t)g_vision_color,
                      (const char *)g_vision_reason);
}

/* ============ 传感�? + 摄像�? 驱动的分拣流�?(�?�?) ============ */
/* 查PG2(�?级还有没有料, 有料则跳过投�?) �? 舵机�?投料 �? �?级跑 �?
 * 中间传感�?(PG0)�?测到�? �? �?级停、二级跑 �? 料到舵机二口(传感�?3 PG1) �? 二级�? �?
 * 料停稳了�? CAPTURE 触发两台摄像头识�? �? 收齐两路 RESULT 融合出形�?+颜色 �? 转盘按形�?+颜色转到对应格位 �?
 * 舵机二投�? �? 转盘回原�? �? 二级�?, �?轮完成�??
 * 转盘格位�?"第一次遇到的形状+颜色组合占一号格, 完全�?致才同格, 不同则依次下�?�?"动�?�分配�?? */

static void camera_sorting_demo_once(void)
{
    float   angle;

    oled_show_stage("LOAD CHECK");   /* 未发送成�?(还没触发) */

    /* �? �?级传送带"有无物料"�?�?(PG2): 有物�? �? 舵机�?不动, 直接跑一�?;
     *    无物�? �? 舵机�?�? 20° 投一个新料�?? */
    if (HAL_GPIO_ReadPin(CONVEYOR1_LOAD_PORT, CONVEYOR1_LOAD_PIN) != CONVEYOR1_LOAD_LEVEL) {
        servo1_open_90();
        HAL_Delay(2000);
        servo1_reset();
    }

    /* �? �?�?: 上料, 持续�? */
    oled_show_stage("C1 RUN");
    conveyor_start(CONVEYOR_1);

    /* �? �? PG0 的有效电平稳�? 40 ms，再把物料交给二级传送带�?
     *    30 s 内没有物料就停掉�?级并结束本轮，避免空带永久运行�?? */
    oled_show_stage("WAIT PG0");
    if (!ir_wait_for_object(IR_SENSOR_TIMEOUT_MS))
    {
        conveyor_stop(CONVEYOR_1);
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);
        return;
    }
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);
    conveyor_stop(CONVEYOR_1);
    conveyor_start(CONVEYOR_2);
    oled_show_stage("C2 WAIT PG1");

    /* ?????????? 10 ????????????? */
    if (!servo2_wait_for_object(SERVO2_SENSOR_TIMEOUT_MS))
    {
			 HAL_Delay(1000);
        conveyor_stop(CONVEYOR_2);
			 HAL_Delay(500);
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);
        oled_show_stage("NO PG1 LOAD");
        return;
    }
					 HAL_Delay(1000);
    conveyor_stop(CONVEYOR_2);
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);
					 HAL_Delay(1000);

    /* �? 料停稳了才发触发: 传感�?3�?测到物体 �? �? CAPTURE 给两台摄像头识别,
     *    等两�? RESULT 融合出形�?+颜色�? */
    oled_show_stage("CAM SEND 1N");   /* 发�?�成�?(已发 CAPTURE, 等摄像头回传) */
    if (vision_capture(20000) < 0) {
        angle = 0.0f;               /* 超时没收�? �? 这轮不动转盘 */
    } else {
        angle = vision_slot_angle(g_vision_shape, g_vision_color);
        /* screen_show_goods(g_vision_shape, g_vision_color); */   /* 串口�?: 对应图片调到该形状行 */
    }
    oled_show_result();   /* 摄像头回传完 �? 显示原始结果、融合结果和规则原因 */

    if (angle <= 0.0f) {
        int32_t vision_diag[6];
        vision_debug_get(vision_diag);
        oled_show_camera_diagnostics(vision_diag);
    }

    if (angle > 0.0f) {
        oled_show_stage("TURN TABLE");
#if SORTING_TURNTABLE_ENABLED
        turntable_rotate_to(angle);
#endif

    /* �? 舵机二投�? �? 回原�? �? 停一�?(时间可调) */
    oled_show_stage("SERVO2 DROP");
    servo2_open_90();
    servo2_reset();
    HAL_Delay(3000);

    /* �? 转盘回软件原�? */
    oled_show_stage("TURN HOME");
#if SORTING_TURNTABLE_ENABLED
    turntable_rotate_to(0.0f);
#endif

    /* �? 二级�?, �?轮分拣完�? */
        screen_show_goods(g_vision_shape, g_vision_color);
        oled_show_stage("DONE");
    }

    else {
        oled_show_stage("NO CAM DROP");
        /* 20 ??????????????????????????????? */
        servo2_open_90();
        servo2_reset();
        HAL_Delay(3000);
    }

    conveyor_stop(CONVEYOR_2);
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);
}

/* 中间传感�?(PG0): 使用原始硬件定义，遮挡时输入为低电平�?
 * 电平约定: 有物�? = 低电�?(IR_OBJECT_LEVEL = GPIO_PIN_RESET)�?
 * �?以配成上拉，空闲时保持高电平。若现场电平相反，需同步修改
 * IR_OBJECT_LEVEL 和此处的上拉配置�? */
static void ir_sensor_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Pin  = IR_SENSOR_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IR_SENSOR_PORT, &gpio);
}

/* 测试: �?级传送带�?直跑, 传感�?(PG0)�?测到物体(遮挡) �? �?级马上停�?
 * 目的: 验证 PG0 和传感器的接线�?�以�?"传感�? �? �?级停"这个动作�?
 * 等待�? PC13 灯跟�?传感器走: 没物�?=�?, 有物�?=�?, 肉眼就能看触发时机�??
 * (想连二级�?起跑, 把下�? conveyor_start(CONVEYOR_2) 打开) */
static void sensor_pg0_test(void)
{
    conveyor_start(CONVEYOR_1);   /* �?级先跑起�? */
//    conveyor_start(CONVEYOR_2);   /* 可�??: 二级�?起跑 */

    while (HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) != IR_OBJECT_LEVEL)
    {
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);   /* 没物�?=灯灭 */
        HAL_Delay(5);
    }

    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);        /* 有物�?=灯亮 */
    conveyor_stop(CONVEYOR_1);      /* �?级马上停 */
    HAL_Delay(3000);                /* �? 3 秒看清楚, 下一轮重新跑 */
}

/* 测试: 漫反射传感器(PG1) + 摄像�? + 转盘 + 舵机�? 联动�?
 * 漫反射检测到物体 �? 给摄像头�? '1' 识别 �? 转盘先转到指定格�? �? 舵机二投�? �? 转盘回位�?
 * (顺序: 转盘先转到位、舵机二再投�?, 否则料会掉进当前格�?�不是目标格)
 * 等待�? PC13 灯跟�?漫反射走: 没物�?=�?, 有物�?=亮�?? */
static void sensor_camera_servo_test(void)
{
    float angle;

    oled_show_status(0);   /* 未发送成�?(还没触发) */

    /* �? 等漫反射传感�?(PG1)�?测到物体(料停在舵机二�?/摄像头下) */
    while (HAL_GPIO_ReadPin(SERVO2_SENSOR_PORT, SERVO2_SENSOR_PIN) != SERVO2_OBJECT_LEVEL)
    {
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);   /* 没物�?=灯灭 */
        HAL_Delay(5);
    }
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);        /* 有物�?=灯亮 */

    /* �? �? CAPTURE 给两台摄像头识别, 等两�? RESULT 融合出形�?+颜色 */
    oled_show_status(1);   /* 发�?�成�?(已发 CAPTURE, 等摄像头回传) */

    if (vision_capture(15000) < 0) {
        angle = 0.0f;               /* 超时没收�? �? 这轮不动转盘 */
    } else {
        angle = vision_slot_angle(g_vision_shape, g_vision_color);
        /* screen_show_goods(g_vision_shape, g_vision_color); */   /* 串口�?: 对应图片调到该形状行 */
    }
    oled_show_result();   /* 摄像头回传完 �? 显示原始结果、融合结果和规则原因 */

    if (angle > 0.0f) {   /* 识别出有效结�?(分配了格�?)才转�? + 舵机二投�? */
        /* �? 转盘先转到指定格�?(把目标格挪到舵机二口下面) */
#if SORTING_TURNTABLE_ENABLED
        turntable_rotate_to(angle);
#endif

        /* �? 舵机二打�?(投料) �? 回位 �? 停一�? */
        servo2_open_90();
        servo2_reset();
        HAL_Delay(3000);

        /* �? 目标板的转盘接口是绝对角度，回原位必须明确发 0°�? */
#if SORTING_TURNTABLE_ENABLED
        turntable_rotate_to(0.0f);
#endif
    }
}

/* 测试: 分拣前半�? —�?? 舵机�?投料 �? �?级传送带�? �? PG0 �?测到物体 �? �?级停�?
 * 目的: 单独验证"投料 �? 运输 �? 中间传感器识�? �? �?级停"这一�?, 不涉及二�?/摄像�?/转盘�?
 * 每轮: 先看 PG2(�?级有无物�?), 无料则舵机一�? 20° 投一个新�?; �?级跑起来�?
 * �? PG0 �?测到物体(遮挡)就停�?�?, �? 3 秒看清楚再进下一轮�?? */
static void sorting_first_half_test(void)
{
    /* �? �?�?"有无物料"�?�?(PG2): 有料 �? 舵机�?不动; 无料 �? 舵机�?�? 20° 投一个新�? */
    if (HAL_GPIO_ReadPin(CONVEYOR1_LOAD_PORT, CONVEYOR1_LOAD_PIN) != CONVEYOR1_LOAD_LEVEL) {
        servo1_open_90();
        HAL_Delay(2000);
        servo1_reset();
    }

    /* �? �?�?: 上料, 持续�? */
    conveyor_start(CONVEYOR_1);

    /* �? 等中间传感器(PG0)�?测到物体(遮挡) �? �?级停。等待时 PC13 灯跟�?传感器走 */
    while (HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) != IR_OBJECT_LEVEL)
    {
        HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_OFF);   /* 没物�?=灯灭 */
        HAL_Delay(5);
    }
    HAL_GPIO_WritePin(IR_LED_PORT, IR_LED_PIN, IR_LED_ON);        /* 有物�?=灯亮 */
    conveyor_stop(CONVEYOR_1);      /* �?级马上停 */
    HAL_Delay(3000);                /* �? 3 秒看清楚, 下一轮重新跑 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  g_boot_stage = 1U;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  g_boot_stage = 2U;

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  g_boot_stage = 3U;
#if SERVO_STANDALONE_TEST
  /* Test before any UART/DMA, sensor, screen or turntable initialization. */
  g_boot_stage = 30U;
  ServoHwTest_Init();
  g_boot_stage = 31U;
  while (1) {
    ServoHwTest_Task();
  }
#endif
  /* TIM4 generates PB6/PB7 PWM even if a later peripheral init cannot finish. */
  Servo_HW_Init();
  g_boot_stage = 4U;
  /* ⚠️ 必须在任�?? MX_xxx_UART_Init() 之前�?? DMA 时钟打开�??
   * �?? UART �?? HAL_UART_MspInit() 里会�?? HAL_DMA_Init() 配置 DMA 寄存�??,
   * 而外设时钟没�??时这些写入会被�?�线直接丢弃 —�?? 偏偏 HAL_DMA_Init() 读回 0
   * 认为 EN 位已�??, 返回 HAL_OK, 不报错�?�结�?? CHSEL/DIR/MINC 全是复位�??,
   * UART4 �??个字节都发不出去, 且完全静默�??
   * 放在 USER CODE 区是为了 CubeMX 重新生成代码时不会被冲掉�?? */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  g_boot_stage = 10U;
  MX_GPIO_Init();
  g_boot_stage = 11U;
  MX_UART4_Init();
  g_boot_stage = 12U;
  MX_USART1_UART_Init();
  g_boot_stage = 13U;
  MX_USART2_UART_Init();
  g_boot_stage = 14U;
  MX_DMA_Init();
  g_boot_stage = 15U;
  MX_USART3_UART_Init();
  g_boot_stage = 16U;
  /* USER CODE BEGIN 2 */
	/* ---- 舵机(PB6/PB7, TIM4): 初始化硬�?? + 两个舵机句柄 ---- */
	Servo_Init(&g_servo[0], 0, 270.0f);   /* 通道0(PB6), 270° 舵机 */
	Servo_Init(&g_servo[1], 1, 180.0f);   /* 通道1(PB7), 180° 舵机 */
	Servo_SetAngle(&g_servo[0], servo1_home_deg);   /* 上电�?? home �?? */
	Servo_SetAngle(&g_servo[1], servo2_home_deg);
  g_boot_stage = 20U;

  /* Start hardware PWM before any motor-bus initialization or waits. */
  zdt_port_init();
  g_boot_stage = 21U;
  zdt_dbg_set(false);
#if SORTING_TURNTABLE_ENABLED
  if (!turntable_wait_boot_home()) {
    oled_init();
    oled_show_stage("HOME ERROR");
    while (1) { HAL_Delay(100U); }
  }
#endif

	led_init();          /* 板载LED(PC13): 调试信号�?? */
	led_blink_times(3, 80, 80);   /* 上电自检: PC13 快闪 3 �?? = 新固件在�??(烧录/复位后最先能看到的信�??) */
	vision_init();       /* 视觉(�? K230, UART5 庐山 + USART2 亚博, 115200): 使能 RXNE 接收中断 */

	oled_init();         /* OLED 状�?�屏(PB8 SCL / PB9 SDA): 显示发�?�成�?/未发送成�? */
	oled_show_status(0); /* 上电默认: 未发送成�? */

	screen_init();       /* 串口�?(USART6 PC6/PC7, 115200): 识别到形�?+颜色后调对应图片到该形状�? */

	conveyor1_load_sensor_init();  /* �?级传送带"有无物料"传感�?(PG2, F103版PB5) */
	ir_sensor_init();          /* 中间传感�?(PG0): 输入+上拉 */
	Sensor_Init();             /* 传感器引脚配置完成后再采样初始电�? */

  /* ===== 串口1 调试台（暂时不用，先注释保留�??=====
   * 以后想接 USB-TTL 看收�??/发指令时，把下面 3 行打�??，再�?? while(1) 里打�??
   * uart1_task(); �?? zdt_dbg_poll(); 即可�??
  rt_ringbuffer_init(&uart1_ringbuffer, ringbuffer1_pool, sizeof(ringbuffer1_pool));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  ================================================== */

  /* 电机总线 DMA 接收(含空闲中�??), 必须�?? MX_DMA_Init 之后�??
   * 即使现在不读回电机状�??, 也要初始�?? —�?? 空闲中断会清�?? TX 完成标志, 发�?�才不卡�?? */
  zdt_dbg_set(false);  // 不接串口调试, 关掉 HEX 打印(否则会往 USART1 �??, 无意�??)

  /* Initialize the UART4 handle for the optional debug task, but do not move it. */
  zdt_init(&chassis, ZDT_FW_EMM, 0, zdt_port4_send);  // address 0 = broadcast

  /* 传�?�带应用使用 USART3 (PE0 控制 RS485 DE)�??
   * 电机 ID 和�?�度集中�?? APP/conveyor_app/conveyor_app.h 中修改�?? */
  conveyor_init();
  //conveyor_start(CONVEYOR_1);   // 串口↔转盘联通测试期间先不跑传�?�带(和F103版一�??)
#if SORTING_TURNTABLE_ENABLED
  //conveyor_start(CONVEYOR_2);

  turntable_init();   /* 转盘(UART4, ID1): 初始化并使能。使能是运行时命令�?�不存flash, 每次上电必须重发 */
  turntable_homing();   /* 设当前位置为软件原点 0�?: 之后 turntable_rotate_to 按绝对角度转/回位 */

  /* 底盘 UART4 测试保留为示�??, 当前传�?�带调试时不发�?�底盘命令�??
   * �??要测试底盘时再取消下面代码的注释, 并确认底盘已接在 UART4�?? */
#endif
  /*
  zdt_enable(&chassis, true);
  HAL_Delay(20);
  zdt_run_vel(&chassis, ZDT_DIR_CW, 10, 100);
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  g_boot_stage = 100U;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Sensor-driven sorting (one round): servo1 drop -> conveyor1 run ->
     * middle sensor (PG0, ported from F103 PB13) detects the part ->
     * conveyor1 stop / conveyor2 run -> camera classify -> turntable ->
     * servo2 drop -> turntable return -> conveyor2 stop. */
//		servo2_open_90();
//		HAL_Delay(500);  
//    servo2_reset();
		camera_sorting_demo_once();
//		sorting_first_half_test();
//	 conveyor_start(CONVEYOR_1);   // 串口↔转盘联通测试期间先不跑传�?�带(和F103版一�??)
//conveyor_start(CONVEYOR_2);
//		sensor_pg0_test();
//    HAL_Delay(500);   /* brief pause between rounds */
//    Sensor_Update();   // 必须每轮调用�??�??, 才会做边沿检测并更新标志
//    /* ===== TURNTABLE TEST: one full turn on every power-up / reset ===== */
//    {
//        static uint8_t turntable_spun = 0;
//        if (!turntable_spun) {
//            turntable_spun = 1;
//            turntable_rotate_rel(360.0f);   /* blocks until one full turn done */
//        }
//        HAL_Delay(50);
//    }

    // 串口1 调试台打�??时，把下面两行取消注释：
    // uart1_task();      // 解析串口1收到�?? #指令@
    // zdt_dbg_poll();    // 把中断里存下的电机应答帧打印出来

    // 传�?�带已在初始化阶段进入�?�度模式; �??要停机时调用 conveyor_stop_all()�??

//    if (Sensor_Is_Goods_Coming()) {
//        // 中间传感器检测到物体经过(高→�??) —�?? 在这里触发分拣动�??
//    }
//    if (Sensor_Is_Goods_Dropped()) {
//        // 料盒传感器检测到物体掉落到位(高→�??) —�?? 在这里计�??/进入下一�??
//    }

    // TODO: 这里放你的电机控�??/状�?�机逻辑
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

