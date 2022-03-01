#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"

#define TSL_SI    PCout(3)   //SI  
#define TSL_CLK   PBout(3)   //CLK 

u16 Get_Adc(u8 ch);
void Dly_us(void);
void RD_TSL(void); 
void CCD(void);
void ccd_Init(void);
char binToHex_high(u8 num);
char binToHex_low(u8 num);
void slove_data(void);
void sendToPc(void);
#endif 


