#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"
void TIM7_Init(u16 arr, u16 psc);
void TIM7_IRQHandler(void);
#endif

