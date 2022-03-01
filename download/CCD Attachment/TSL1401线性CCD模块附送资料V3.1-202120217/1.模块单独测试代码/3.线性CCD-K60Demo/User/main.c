
#include "include.h"
u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //线性CCD FLASH相关


//主函数
void main(void)
{
    PLL_Init(PLL180);//时钟设置180M
    NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC中断设置
    systime.init();//延时
    OLED_Init();//用于OLED显示
    CCD_Init();//CCD初始化
    MyPIT_Init();//定时器中断，在定时器中断里读取CCD数据

while(1)
{
  oled_show();
  delay_ms(50);
  }
}

