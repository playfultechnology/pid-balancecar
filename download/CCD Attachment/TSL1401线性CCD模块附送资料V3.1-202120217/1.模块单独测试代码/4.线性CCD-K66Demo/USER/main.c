
#include "include.h"



u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //����CCD FLASH���


void main(void)
{
  /* ��ʼ��PLLΪ180M */
  PLL_Init(PLL180);      //ʱ������
  NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC�ж�����
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

