#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//�����˲����ṹ��
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //�־� ���ֳ�Ϊ���־�
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //��� ���ֳ�Ϊ�����	
  int GearRatio;           //Motor_gear_ratio //������ٱ�
  int EncoderAccuracy;     //Number_of_encoder_lines //����������(����������)
  float WheelDiameter;     //Diameter of driving wheel //������ֱ��	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //ȫ����С����ת�뾶	
}Robot_Parament_InitTypeDef;

// Encoder structure
//�������ṹ��
typedef struct  
{
  int A;      
  int B; 
	int C; 
	int D; 
}Encoder;

//Wheelspacing, Mec_Car is half wheelspacing
//�־� ������һ��


#define MEC_BS_wheelspacing         0.098f //����2021.03.30						//����ʽ����
#define MEC_wheelspacing 						0.081f													//������ʽ����

#define Four_Mortor_BS_wheelSpacing 0.098f														//����ʽ����		
#define Four_Mortor_wheelSpacing 		0.081f												//������ʽ����		
//#define Tank_wheelSpacing        0.235f

//Axlespacing, Mec_Car is half axlespacing
//��� ������һ��
#define MEC_BS_axlespacing           0.087f															//����ʽ����
#define MEC_axlespacing           		0.085f															//������ʽ����

#define Four_Mortor__BS_axlespacing  0.087f																//����ʽ����
#define Four_Mortor__axlespacing  	0.085f													//������ʽ����
//#define Tank_axlespacing          0.222f

//Motor_gear_ratio
//������ٱ�
#define   HALL_30F    30
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47

//Number_of_encoder_lines
//����������
#define		Photoelectric_500 500
#define	  Hall_13           13

//Mecanum wheel tire diameter series
//������ֱ̥��
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 
//Omni wheel tire diameter series
//�־�ȫ����ֱ��ϵ��
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//Black tire, tank_car wheel diameter
//��ɫ��̥���Ĵ�����ֱ��
#define	  Black_WheelDiameter   0.067f
//#define	  Tank_WheelDiameter 0.047
#define	  Tank_WheelDiameter 0.043

//Rotation radius of omnidirectional trolley
//ȫ����С����ת�뾶
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

//The encoder octave depends on the encoder initialization Settings
//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples  4
//Encoder data reading frequency
//���������ݶ�ȡƵ��
#define   CONTROL_FREQUENCY 100

#define PI 3.1415f  //PI //Բ����

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
