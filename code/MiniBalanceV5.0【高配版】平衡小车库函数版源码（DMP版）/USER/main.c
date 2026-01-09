#include "stm32f4xx.h"
#include "sys.h"  
u8 Way_Angle=1;                              // Algorithm for obtaining angle: 1: Quaternion 2: Kalman Filter 3: Complementary Filter
// Qian = Fwd, Hou = Back, Sudu = Speed
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; // Variables related to Bluetooth remote control
u8 Flag_Stop=1,Flag_Show=0,Flag_Hover=0;    // Stop flag and display flag. Default is stopped and display is on.
int Encoder_Left,Encoder_Right;             // Pulse count for left and right encoders
int Moto1,Moto2;                            // Motor PWM variables.
int Temperature;                            // Display Temperature
int Voltage;                                // Battery Voltage Sampling
float Angle_Balance,Gyro_Balance,Gyro_Turn; // Balance angle, balance gyroscope, steering gyroscope
float Show_Data_Mb;                         //Global display variables, used to display the data to be viewed
u32 Distance;                               // Ultrasonic ranging
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //Delay and parameter adjustment variables
float Acceleration_Z;                       //Z-axis accelerometer  
float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;//PID parameters
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash
float Zhongzhi=-4;                          // Mechanical median
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      // RC control receiver variables
int main(void)
{
	delay_init(168);                //=====主频168M
	uart_init(128000);              //=====延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====设置系统中断优先级分组2
	LED_Init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
  OLED_Init();                    //=====OLED初始化
	TIM3_Cap_Init(0XFFFF,84-1);	    //===== Ultrasonic Init
	TIM8_Cap_Init(0XFFFF,168-1);	  //===== RC Control Init
	Encoder_Init_TIM2();            //=====编码器初始化
	Encoder_Init_TIM4();            //=====编码器初始化
	uart2_init(9600);               //=====Serial Port 2 Init
	delay_ms(500);                  //=====延时等待系统稳定
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP 
	Adc_Init();                     //=====模拟量采集初始化
	//NRF24L01_Init();    					//=====初始化NRF24L01  
  //while(NRF24L01_Check());      //=====NRF24L01模块自检，开启之后必须接入NRF24L01模块，程序才继续执行
	CAN1_Mode_Init(1,7,6,3,CAN_Mode_Normal);//=====CAN初始化  
	MiniBalance_PWM_Init(8400,1);   //=====PWM初始化
	MiniBalance_EXTI_Init();        //=====外部中断初始化
  while(1){
			if(Flash_Send==1)        //写入PID参数到Flash,由app控制该指令
				{
					Flash_Write();	
					Flash_Send=0;	
				}	
			if(Flag_Show==0)        	  //使用MiniBalance APP和OLED显示屏
				{
						APP_Show();	
						oled_show();          //===显示屏打开
				}
				else                      //使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
				{
						DataScope();          //开启MiniBalance上位机
				}	
				delay_flag=1;	
				delay_50=0;
				while(delay_flag);	     //通过MPU6050的INT中断实现的50ms精准延时	
	}
}
