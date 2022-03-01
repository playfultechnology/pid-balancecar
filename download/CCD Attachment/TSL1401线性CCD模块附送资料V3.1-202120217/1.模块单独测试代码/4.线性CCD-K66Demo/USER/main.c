
#include "include.h"



u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //线性CCD FLASH相关


void main(void)
{
  /* 初始化PLL为180M */
  PLL_Init(PLL180);      //时钟设置
  NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC中断设置
  systime.init();

  OLED_Init();
  CCD_Init();
  MyPIT_Init();

while(1)
{
  oled_show();
  delay_ms(50);
  }
}

