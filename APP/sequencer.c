#include "sequencer.h"
#include <stdio.h> // 需要用到 printf 打印文字

// --- 1. 全局变量的本体定义 (这里不能加 extern) ---
uint8_t Flag_Item_Detected = 0; 
uint8_t Flag_Vision_Done = 0;   
uint8_t Vision_Result_ID = 0;   

// --- 2. 状态机主函数实现 ---
void Sequencer_Run(void) {
    // 静态变量，保存当前状态，每次进入函数不会丢失
    static SystemState_t current_state = STATE_FEEDING;

    switch (current_state) {
        // ----------------------------------------------------
        case STATE_FEEDING:
            // 真实动作：HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            printf("[状态] 正在进料，一级传送带运行中...\r\n");
            
            // 如果传感器被触发了 (这个标志位会在外部中断里被置 1)
            if (Flag_Item_Detected == 1) {
                printf("  -> [触发] 光电传感器检测到物料掉落！\r\n");
                printf("  -> [动作] 一级传送带【急停】。\r\n");
                Flag_Item_Detected = 0; // 清除标志位，防止重复触发
                current_state = STATE_TRANSPORT; // 跳转到下一状态
            }
            break;

        // ----------------------------------------------------
        case STATE_TRANSPORT:
            // 真实动作：开启二级电机，延时，关闭电机。向串口1发指令。
            printf("[状态] 二级传送带启动，将物料运往摄像头...\r\n");
            
            HAL_Delay(1000); // 模拟物料移动耗时
            
            printf("  -> [动作] 物料到达视觉区，二级传送带【停止】。\r\n");
            printf("  -> [动作] 串口发送指令，要求视觉模块拍照。\r\n");
            current_state = STATE_WAIT_VISION;
            break;

        // ----------------------------------------------------
        case STATE_WAIT_VISION:
            // 真实动作：主循环在这里空转，等待串口接收中断改变标志位。
            printf("[状态] 正在等待 AI 视觉模块返回识别结果...\r\n");
            
            // 假设串口接收到了数据 (这个标志位会在串口接收中断里被置 1)
            if (Flag_Vision_Done == 1) {
                printf("  -> [触发] 收到视觉结果！物品类别为: %d 类。\r\n", Vision_Result_ID);
                Flag_Vision_Done = 0; // 清除标志位
                current_state = STATE_ROTATE_BOX;
            }
            break;

        // ----------------------------------------------------
        case STATE_ROTATE_BOX:
            // 真实动作：计算角度，控制步进电机转动
            printf("[状态] 底部转盘开始寻址...\r\n");
            printf("  -> [动作] 步进电机旋转至 %d 号储物盒。\r\n", Vision_Result_ID);
            
            HAL_Delay(1000); // 模拟转盘旋转耗时
            current_state = STATE_DROP;
            break;

        // ----------------------------------------------------
        case STATE_DROP:
            // 真实动作：开启二级传送带，延时，停止
            printf("[状态] 目标盒子已就位，开始投料...\r\n");
            printf("  -> [动作] 二级传送带启动，物料掉入 %d 号盒。\r\n", Vision_Result_ID);
            
            HAL_Delay(800); // 模拟物料掉落耗时
            current_state = STATE_UPDATE_UI;
            break;

        // ----------------------------------------------------
        case STATE_UPDATE_UI:
            // 真实动作：调用 SPI 刷屏，清零超时看门狗定时器
            printf("[状态] 投递完成！正在刷新屏幕数据...\r\n");
            printf("  -> [动作] 重置 15 秒超时计时器。\r\n");
            
            printf("================ 一次分拣流程结束 ================\r\n\n");
            HAL_Delay(2000); // 停顿一下，防止串口打印太快看不清
            
            current_state = STATE_FEEDING; // 完美闭环，回到初始状态
            break;
    }
}
