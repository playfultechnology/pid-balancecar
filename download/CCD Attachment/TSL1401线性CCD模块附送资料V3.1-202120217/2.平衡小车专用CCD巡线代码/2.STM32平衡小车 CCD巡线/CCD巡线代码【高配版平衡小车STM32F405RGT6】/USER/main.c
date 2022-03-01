#include "stm32f4xx.h"
#include "sys.h"  
u8 Way_Angle=3;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0,Flag_Hover=0;    //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //延时和调参等变量
float Acceleration_Z;                       //Z轴加速度计  
float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;//PID参数
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组
float Zhongzhi=10;                          //机械中值
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //航模遥控接收变量
u16 ADV[128]={0};                           //CCD数组
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //线性CCD相关
int main(void)
{
	delay_init(168);                //=====主频168M
	uart_init(128000);              //=====延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====设置系统中断优先级分组2
	LED_Init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
  OLED_Init();                    //=====OLED初始化
	Encoder_Init_TIM2();            //=====编码器初始化
	Encoder_Init_TIM4();            //=====编码器初始化
	uart2_init(9600);               //=====串口2初始化
	delay_ms(500);                  //=====延时等待系统稳定
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP 
	Adc_Init();                     //=====模拟量采集初始化  
	MiniBalance_PWM_Init(8400,1);   //=====PWM初始化
	TIM3_Int_Init(50-1,8400-1);	    //定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数50次为5ms     
  while(1){
		oled_show();            //===显示屏打开
	}
}
