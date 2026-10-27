#ifndef __USART12_APP_H
#define __USART12_APP_H

#include "mydefine.h"

void uart1_task(void);
void uart2_task(void);
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

#endif
