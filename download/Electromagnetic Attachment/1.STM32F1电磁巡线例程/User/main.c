#include "stm32f10x.h"
#include "sys.h"


int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//电磁巡线相关



int main(void)
  {
		delay_init();	    	        //=====延时函数初始化	
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
		LED_Init();                     //=====初始化与 LED 连接的硬件接口
		MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
		OLED_Init();					//=====OLED初始化
		uart_init(9600);				//=====串口1初始化
	  
		ele_Init();  //=====电磁传感器初始化	

		
	while(1)
	{
		int  sum;
		
	
		Sensor_Left=Get_Adc(11);                //采集左边电感的数据
		Sensor_Right=Get_Adc(13);               //采集右边电感的数据
		Sensor_Middle=Get_Adc(12);              //采集中间电感的数据
		sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //归一化处理
		Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //求偏差
		
		
		oled_show();
		
		printf("      Left: %d\r",Sensor_Left);
		printf("      Middle: %d\r",Sensor_Middle);
		printf("      Right: %d\r",Sensor_Right);
		printf("      Sensor: %d\r\n",Sensor);
		
		delay_ms(50);
		
	  } 
}

