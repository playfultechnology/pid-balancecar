#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u16 ADV[128]={0};              
u8 CCD_Zhongzhi,CCD_Yuzhi;   //线性CCD  相关
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	KEY_Init();                     //=====按键初始化
	OLED_Init();                    //=====OLED初始化
	delay_ms(300);                  //=====延时启动
	uart_init(72,128000);           //=====初始化串口1
	ccd_Init();                     //=====电池电压采样adc初始化
	Timer_Init(49,7199);             //=====定时中断初始化 
	while(1)
		{     
			oled_show();          //===显示屏打开
	}
}
