#ifndef __MYDEFINE_H__
#define __MYDEFINE_H__

//C语言的
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h> 
#include <stdbool.h>
#include <math.h>

//系统的（用到了什么就解除什么的注释）
#include "main.h"
#include "gpio.h"
#include "usart.h"


//自己写的（用到了什么就解除什么的注释）
//#include "./scheduler_app/scheduler_app.h"
#include "./ioout_app/ioout_app.h"
#include "./ebtn_app/ebtn_app.h"
#include "./usart12_app/usart12_app.h"  //usart功能头文件引入
#include "./zdt_motor/zdt_motor.h"      // ZDT闭环步进电机驱动库
#include "./zdt_motor/zdt_motor_port.h" // ZDT电机UART移植层(双串口
#include "sensor.h"
//#include "./usart1_app/usart1_app.h"
//#include "./usart2_app/usart2_app.h"
//#include "./usart3_app/usart3_app.h"

//组件库（用到了什么就解除什么的注释）
#include "ebtn.h"
#include "ringbuffer.h"
#include "pt.h"


//外部引用的变量


/* 串口1 */
extern uint8_t uart1_rx_dma_buffer[128]; //dma的接收的, 只接受但是不处理
extern uint8_t uart1_process_buffer[128];//这个就是dma接收完毕之后在传给这个数组, 解析操作只在这个里面进行;
extern struct rt_ringbuffer uart1_ringbuffer;
extern uint8_t ringbuffer1_pool[128];
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/* 串口2 */
extern uint8_t uart2_rx_dma_buffer[128]; // UART2的DMA接收缓冲区
extern uint8_t uart2_process_buffer[128]; // UART2的数据处理缓冲区
extern struct rt_ringbuffer uart2_ringbuffer;       // UART2的环形缓冲区结构体
extern uint8_t ringbuffer2_pool[128];        // UART2环形缓冲区的内存池
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


/* Legacy ZDT handles (defined in main.c). conveyor_app uses private handles. */
extern zdt_t conveyor1;
extern zdt_t conveyor2;
extern zdt_t chassis;






#endif
