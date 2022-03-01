#ifndef INCLUDE_H_
#define INCLUDE_H_

/*
 * ����Cortex-M�ں˵�ͨ��ͷ�ļ�
 */
#include    <stdio.h>
#include    <string.h>
#include    <stdlib.h>
#include    <stdint.h>
#include    <math.h>

#include "common.h"
#include "MK60N512VMD100.h"   /* �Ĵ���ӳ��ͷ�ļ� */
#include "core_cm4.h"         /* �ں��ļ����������ж����ȼ� */

/*----------------------------------------------------------------
                   �ײ������ļ�
------------------------------------------------------------------*/
#include "MK60_ADC.h"              /* ����ADCģ�� */
#include "MK60_GPIO.h"             /* ����GPIOģ�� */
#include "MK60_GPIO_Cfg.h"         /* ����GPIO */
#include "MK60_PLL.h"              /* ����ʱ��Ƶ������ */
#include "MK60_UART.h"             /* ���ڴ���ģ�� */
#include "MK60_PIT.h"              /* ����PIT��ʱ��ģ�� */
#include "MK60_FTM.h"              /* ����FTM��ʱ��ģ�� */
#include "MK60_CMT.h"              /* ����CMT��ʱ��ĳ�� */
#include "MK60_IIC.h"              /* ����IICģ�� */
#include "MK60_DMA.h"              /* ����DMAģ�� */
#include "MK60_LPTMR.h"            /* ����LPTMR��ʱ��ģ�� */
#include "MK60_WDOG.h"             /* ���ڿ��Ź� */
#include "MK60_SYSTICK.h"          /* systick �ں˶�ʱ�� */

#include "LED.h"                /* LED ���� */
#include "KEY.h"                /* KEY ���� */
#include "PIT.h"                /* PIT ���� */
#include "UART.h"               /*   ����   */
#include "show.h"               /* �� ʾ ���� */
#include "oled.h"               /* OLED ���� */
#include "control.h"            /* �û� ���� */
#include "ccd.h"                /* CCD������*/
#include "ele.h"                /* ���Ѳ�� */
#include "pstwo.h"              /*  PS2�ֱ� */
#include "motor.h"              /*  ������� */

/****************ȫ�ֱ���*********************/

extern int Flag_Direction; //����ң����صı���
extern u8 Flag_Way,Flag_Show,Flag_Stop,Flag_Next;   //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
extern int Encoder_Left,Encoder_Right;             //���ұ��������������
extern int Encoder_A_EXTI,Encoder_Temp;
extern float Velocity,Velocity_Set,Angle,Angle_Set;
extern int Motor_A,Motor_B,Servo,Target_A,Target_B;  //���PWM���� Ӧ��Motor�� ��Moto�¾�
extern int Voltage;                                //��ص�ѹ������صı���
extern float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
extern u8 delay_50,delay_flag; //��ʱ�͵��εȱ���
extern float Velocity_KP,Velocity_KI;	       //�ٶȿ���PID����
extern u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send,Bluetooth_Velocity,APP_RX;     //����CCD���
extern u16 PID_Parameter[10],Flash_Parameter[10];

extern int ELE_Sensor,Sensor_Left,Sensor_Middle,Sensor_Right;;//���Ѳ�����
extern uint16 ADV[128];         //==CCD��ȡ���ݱ���

/****************ȫ�ֱ���*********************/

#endif