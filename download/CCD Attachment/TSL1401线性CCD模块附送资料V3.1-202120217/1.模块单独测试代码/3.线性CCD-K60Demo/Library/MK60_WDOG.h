
#ifndef __WDOG_H__
#define __WDOG_H__

void WDOG_SetTime(uint32 ms);//设置喂狗时间

void WDOG_Feed(void);//喂狗

void WDOG_Unlock(void);//解锁

void WDOG_Enable(void);//使能

void WDOG_Disable(void);//失能

#endif
