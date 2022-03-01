#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"

void KEY_Init(void);
u8 click(void);
void Delay_ms(void);
u8 click_N_Double (u8 time);
u8 click_N_Double_ICM20948 (u8 time);
u8 Long_Press(void);
u8  select(void);
/*--------KEY control pin--------*/
#define KEY_PORT	GPIOD
#define KEY_PIN		GPIO_Pin_8
#define KEY			PDin(8) 
/*----------------------------------*/

#endif 
