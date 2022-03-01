#ifndef INCLUDE_H_
#define INCLUDE_H_

/*
 * 包含Cortex-M内核的通用头文件
 */
#include    <stdio.h>
#include    <string.h>
#include    <stdlib.h>
#include    <stdint.h>
#include    <math.h>

#include "common.h"
#include "MK60N512VMD100.h"   /* 寄存器映像头文件 */
#include "core_cm4.h"         /* 内核文件用于设置中断优先级 */

/*----------------------------------------------------------------
                   底层驱动文件
------------------------------------------------------------------*/
#include "MK60_ADC.h"              /* 用于ADC模块 */
#include "MK60_GPIO.h"             /* 用于GPIO模块 */
#include "MK60_GPIO_Cfg.h"         /* 用于GPIO */
#include "MK60_PLL.h"              /* 用于时钟频率设置 */
#include "MK60_UART.h"             /* 用于串口模块 */
#include "MK60_PIT.h"              /* 用于PIT定时器模块 */
#include "MK60_FTM.h"              /* 用于FTM定时器模块 */
#include "MK60_CMT.h"              /* 用于CMT定时器某块 */
#include "MK60_IIC.h"              /* 用于IIC模块 */
#include "MK60_DMA.h"              /* 用于DMA模块 */
#include "MK60_LPTMR.h"            /* 用于LPTMR定时器模块 */
#include "MK60_WDOG.h"             /* 用于看门狗 */
#include "MK60_SYSTICK.h"          /* systick 内核定时器 */

#include "LED.h"                /* LED 驱动 */
#include "KEY.h"                /* KEY 驱动 */
#include "PIT.h"                /* PIT 驱动 */
#include "UART.h"               /*   串口   */
#include "show.h"               /* 显 示 驱动 */
#include "oled.h"               /* OLED 驱动 */
#include "control.h"            /* 用户 代码 */
#include "ccd.h"                /* CCD传感器*/
#include "ele.h"                /* 电磁巡线 */
#include "pstwo.h"              /*  PS2手柄 */
#include "motor.h"              /*  电机控制 */

/****************全局变量*********************/

extern int Flag_Direction; //蓝牙遥控相关的变量
extern u8 Flag_Way,Flag_Show,Flag_Stop,Flag_Next;   //停止标志位和 显示标志位 默认停止 显示打开
extern int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
extern int Encoder_A_EXTI,Encoder_Temp;
extern float Velocity,Velocity_Set,Angle,Angle_Set;
extern int Motor_A,Motor_B,Servo,Target_A,Target_B;  //电机PWM变量 应是Motor的 向Moto致敬
extern int Voltage;                                //电池电压采样相关的变量
extern float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
extern u8 delay_50,delay_flag; //延时和调参等变量
extern float Velocity_KP,Velocity_KI;	       //速度控制PID参数
extern u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send,Bluetooth_Velocity,APP_RX;     //线性CCD相关
extern u16 PID_Parameter[10],Flash_Parameter[10];

extern int ELE_Sensor,Sensor_Left,Sensor_Middle,Sensor_Right;;//电磁巡线相关
extern uint16 ADV[128];         //==CCD读取数据保存

/****************全局变量*********************/

#endif