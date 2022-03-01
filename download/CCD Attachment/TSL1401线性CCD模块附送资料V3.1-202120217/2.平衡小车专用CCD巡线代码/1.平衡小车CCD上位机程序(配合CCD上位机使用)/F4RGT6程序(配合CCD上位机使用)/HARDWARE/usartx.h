#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define FRAME_HEADER_CAR 0X7B //���Ƶ����˶�������֡ͷ
#define FRAME_TAIL_CAR 0X7D //���Ƶ����˶�������֡β
#define FRAME_HEADER_MOVEIT 0XAA //���ƻ�е���˶�������֡ͷ
#define FRAME_TAIL_MOVEIT 0XBB //���ƻ�е���˶�������֡ͷ
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****���ڴ�������Ǽ��ٶȼ��������ݵĽṹ��*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******���ڷ������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		short X_speed;	            //2 bytes //2���ֽ�
		short Y_speed;              //2 bytes //2���ֽ�
		short Z_speed;              //2 bytes //2���ֽ�
		short Power_Voltage;        //2 bytes //2���ֽ�
		Mpu6050_Data Accelerometer; //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;     //6 bytes //6���ֽ�	
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		float X_speed;	            //4 bytes //4���ֽ�
		float Y_speed;              //4 bytes //4���ֽ�
		float Z_speed;              //4 bytes //4���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Control_Str;
}RECEIVE_DATA;

void data_send(void );
void USART3_SEND(void);
void USART1_SEND(void);
void data_transition(void);
void usart2_send(u8 data);
void uart2_init(u32 bound);
int USART2_IRQHandler(void);
void usart3_send(u8 data);
void usart1_send(u8 data);
void uart3_init(u32 bound);
int USART3_IRQHandler(void);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
void uart1_init(u32 bound);
int USART1_IRQHandler(void);

float XYZ_Target_Speed_transition(u8 High,u8 Low);
void CAN_SEND(void);


#endif

