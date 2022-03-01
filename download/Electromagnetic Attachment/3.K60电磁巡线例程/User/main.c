
#include "include.h"


//u8 Flag_Way=0,Flag_Show=0,Flag_Stop=1,Flag_Next;  //停止标志位和 显示标志位 默认停止 显示打开
//int Encoder_Left,Encoder_Right;                  //左右编码器的脉冲计数
//int Encoder_A_EXTI,Flag_Direction;
//int Encoder_Temp;
//float Velocity,Velocity_Set,Angle,Angle_Set;
//int Motor_A,Motor_B,Servo,Target_A,Target_B;  //电机舵机控制相关
//int Voltage;                                //电池电压采样相关的变量
//float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
//u8 delay_50,delay_flag; //延时变量
//float Velocity_KP=12,Velocity_KI=12;	       //速度控制PID参数
//u8 Bluetooth_Velocity=7,APP_RX;                 //蓝牙遥控速度和APP接收的数据
//u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //线性CCD FLASH相关


int ELE_Sensor,Sensor_Left,Sensor_Middle,Sensor_Right;;//电磁巡线相关

//
//u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组





//主函数
void main(void)
{
    /* 初始化PLL为180M */
    PLL_Init(PLL180);      //时钟设置
    NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC中断设置
    systime.init();
    LED_Init();//LED灯

    OLED_Init();
    ELE_Init();


while(1)
{
  ELE_Sensor=(int)(Get_ELE_Bias());//读取电磁巡线传感器偏离中线数据


  oled_show();

//  printf("      Left: %d\r",Sensor_Left);
//  printf("      Middle: %d\r",Sensor_Middle);
//  printf("      Right: %d\r",Sensor_Right);
//  printf("      Sensor: %d\r\n",ELE_Sensor);


  delay_ms(50);
  }
}

