#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PBin(14)	
#define MODE PBin(12)	//模式开关
#define IR1 PAin(8)  	//红外1
#define IR2 PAin(11)	//红外2
void KEY_Init(void);          //按键初始化
u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);
u8  select(void);

#endif 
