#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void usart1_send(u8 data);

#endif


