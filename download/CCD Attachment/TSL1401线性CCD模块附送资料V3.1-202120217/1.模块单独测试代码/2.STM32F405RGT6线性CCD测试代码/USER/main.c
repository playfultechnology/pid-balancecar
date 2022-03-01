#include "stm32f4xx.h"
#include "sys.h"  
u16 ADV[128]={0};              
u8 CCD_Zhongzhi,CCD_Yuzhi;   //线性CCD  相关
int main(void)
{
	delay_init(168);                //=====主频168M
	uart_init(128000);              //=====延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====设置系统中断优先级分组2
	LED_Init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
  OLED_Init();                    //=====OLED初始化
	delay_ms(500);                  //=====延时等待系统稳定
	Adc_Init();                     //=====电池电压采样adc初始化
	TIM3_Int_Init(50-1,8400-1);	    //定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数50次为5ms     
  while(1){
		oled_show();            //===显示屏打开
	}
}
