#include "stm32f4xx.h"
#include "sys.h"  
u8 Flag_Stop=0,delay_50,delay_flag,Flash_Send,PID_Send;                   //停止标志位 50ms精准延时标志位
int Encoder,Encoder_Key,Target_Position=10000,Target_Velocity=40; //编码器的脉冲计数
int Moto;                                             //电机PWM变量 应是Motor的 向Moto致敬	
int Voltage;                                          //电池电压采样相关的变量
float Position_KP=40,Position_KI=0.1,Position_KD=200,Velocity_KP=5,Velocity_KI=5;      //PID系数
float Amplitude_PKP=2,Amplitude_PKI=0.1,Amplitude_PKD=3,Amplitude_VKP=1,Amplitude_VKI=1; //PID调试相关参数
float Menu_MODE=1,Menu_PID=1;  //PID相关标志位
float Angle_Balance;  //角度
u8 Flag_MODE=1;  
int main(void)
{
	delay_init(168);                //=====主频168M
	uart_init(128000);              //=====延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====设置系统中断优先级分组2
	LED_Init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
  OLED_Init();                    //=====OLED初始化
	Encoder_Init_TIM4();            //=====编码器初始化
	delay_ms(500);                  //=====延时等待系统稳定
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP 
	Adc_Init();                     //=====模拟量采集初始化
	MiniBalance_PWM_Init(8400,1);   //=====PWM初始化
			while(Flag_MODE)                //=====让用户选择  运行模式
		{
		  oled_show_once();               //=====临时显示OLED
			if(TIM4->CNT>10500)Menu_MODE=0; //速度模式  向后转动右轮
			if(TIM4->CNT<9500) Menu_MODE=1; //位置模式  向前转动右轮
		  if(TIM4->CNT>10500||TIM4->CNT<9500)Flag_MODE=0,OLED_Clear(),TIM4->CNT=0;
		}
		 Encoder_Init_TIM2();            //=====编码器接口
	  MiniBalance_EXTI_Init();        //=====MPU6050 5ms定时中断初始化
  while(1){
			delay_flag=1;	              //===50ms中断精准延时标志位
			oled_show();                //===显示屏打开	  	
			DataScope();			           //===上位机
			while(delay_flag);          //===50ms中断精准延时  主要是波形显示上位机需要严格的50ms传输周期   	
	}
}
