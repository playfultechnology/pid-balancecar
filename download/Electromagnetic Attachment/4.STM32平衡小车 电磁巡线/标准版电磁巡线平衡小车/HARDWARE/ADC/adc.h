#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define Battery_Ch 6
void Adc_Init(void);
u16 Get_Adc(u8 ch);
u16 Get_Adc2(u8 ch);   
int Get_battery_volt(void);   
#endif 















