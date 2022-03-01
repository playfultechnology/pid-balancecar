#include "robot_select_init.h"

//Initialize the robot parameter structure
//��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺţ�CAR_NUMBER=6
	Divisor_Mode=2048/CAR_NUMBER+3;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //Collect the pin information of potentiometer //�ɼ���λ��������Ϣ	
  if(Car_Mode>5)Car_Mode=5;
		
	switch(Car_Mode)
	{ 
			case 0:       Robot_Init(MEC_BS_wheelspacing,         MEC_BS_axlespacing,           0,                     HALL_30F, Hall_13, Mecanum_75);             //�����ķ��С������ʽ����75mm
			case 1:       Robot_Init(MEC_BS_wheelspacing,         MEC_BS_axlespacing,           0,                     HALL_30F, Hall_13, Mecanum_60);            //�����ķ��С������ʽ����60mm
			case 2:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          				0,                     HALL_30F, Hall_13, Mecanum_75);             //�����ķ��С��������ʽ����75mm
			case 3:	     Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          				0,                     HALL_30F, Hall_13, Mecanum_60);            //�����ķ��С��������ʽ����60m
			case 4:			  Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 				0,                     HALL_30F, Hall_13, Black_WheelDiameter);   //������������ʽ���� 
			case 5:				Robot_Init(Four_Mortor_BS_wheelSpacing, Four_Mortor__BS_axlespacing,  0,                     HALL_30F, Hall_13, Black_WheelDiameter);   //����������ʽ����
	}
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//wheelspacing, Mec_Car is half wheelspacing
	//�־� ���ֳ�Ϊ���־�
  Robot_Parament.WheelSpacing=wheelspacing; 
	//axlespacing, Mec_Car is half axlespacing
  //��� ���ֳ�Ϊ�����	
  Robot_Parament.AxleSpacing=axlespacing;   
 
	//motor_gear_ratio
	//������ٱ�
  Robot_Parament.GearRatio=gearratio; 
	//Number_of_encoder_lines
  //����������(����������)	
  Robot_Parament.EncoderAccuracy=Accuracy;
	//Diameter of driving wheel
  //������ֱ��	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//���(����)ת1Ȧ��Ӧ�ı�������ֵ
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //�������ܳ�	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//wheelspacing, Mec_Car is half wheelspacing
  //�־� ���ֳ�Ϊ���־�  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //axlespacing, Mec_Car is half axlespacing	
  //��� ���ֳ�Ϊ�����	
	Axle_spacing=Robot_Parament.AxleSpacing; 
}

