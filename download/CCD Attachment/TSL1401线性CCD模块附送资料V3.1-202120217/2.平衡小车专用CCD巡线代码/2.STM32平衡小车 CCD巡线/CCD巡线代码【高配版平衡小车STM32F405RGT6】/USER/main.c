#include "stm32f4xx.h"
#include "sys.h"  
u8 Way_Angle=3;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0,Flag_Hover=0;    //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance;                               //���������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //��ʱ�͵��εȱ���
float Acceleration_Z;                       //Z����ٶȼ�  
float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;//PID����
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
float Zhongzhi=10;                          //��е��ֵ
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //��ģң�ؽ��ձ���
u16 ADV[128]={0};                           //CCD����
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //����CCD���
int main(void)
{
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
  OLED_Init();                    //=====OLED��ʼ��
	Encoder_Init_TIM2();            //=====��������ʼ��
	Encoder_Init_TIM4();            //=====��������ʼ��
	uart2_init(9600);               //=====����2��ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP 
	Adc_Init();                     //=====ģ�����ɼ���ʼ��  
	MiniBalance_PWM_Init(8400,1);   //=====PWM��ʼ��
	TIM3_Int_Init(50-1,8400-1);	    //��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����50��Ϊ5ms     
  while(1){
		oled_show();            //===��ʾ����
	}
}
