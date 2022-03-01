#include "stm32f4xx.h"
#include "sys.h"  
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0,Flag_Hover=0;    //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;//PID����
float Zhongzhi;
int main(void)
{
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
  OLED_Init();                    //=====OLED��ʼ��
	Encoder_Init_TIM2();            //=====��������ʼ��
	Encoder_Init_TIM4();            //=====��������ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP 
	MiniBalance_Motor_Init();
	MiniBalance_PWM_Init(8400,1);   //=====PWM��ʼ��
	MiniBalance_EXTI_Init();        //=====�ⲿ�жϳ�ʼ��
  while(1){
	oled_show();          //===��ʾ����
	}
}
