#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void usart1_send(u8 data);

#endif


