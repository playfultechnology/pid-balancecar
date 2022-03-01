#include "stm32f4xx.h"
#include "sys.h"  
u8 Flag_Stop=0,delay_50,delay_flag,Flash_Send,PID_Send;                   //ֹͣ��־λ 50ms��׼��ʱ��־λ
int Encoder,Encoder_Key,Target_Position=10000,Target_Velocity=40; //���������������
int Moto;                                             //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Voltage;                                          //��ص�ѹ������صı���
float Position_KP=40,Position_KI=0.1,Position_KD=200,Velocity_KP=5,Velocity_KI=5;      //PIDϵ��
float Amplitude_PKP=2,Amplitude_PKI=0.1,Amplitude_PKD=3,Amplitude_VKP=1,Amplitude_VKI=1; //PID������ز���
float Menu_MODE=1,Menu_PID=1;  //PID��ر�־λ
float Angle_Balance;  //�Ƕ�
u8 Flag_MODE=1;  
int main(void)
{
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
  OLED_Init();                    //=====OLED��ʼ��
	Encoder_Init_TIM4();            //=====��������ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP 
	Adc_Init();                     //=====ģ�����ɼ���ʼ��
	MiniBalance_PWM_Init(8400,1);   //=====PWM��ʼ��
			while(Flag_MODE)                //=====���û�ѡ��  ����ģʽ
		{
		  oled_show_once();               //=====��ʱ��ʾOLED
			if(TIM4->CNT>10500)Menu_MODE=0; //�ٶ�ģʽ  ���ת������
			if(TIM4->CNT<9500) Menu_MODE=1; //λ��ģʽ  ��ǰת������
		  if(TIM4->CNT>10500||TIM4->CNT<9500)Flag_MODE=0,OLED_Clear(),TIM4->CNT=0;
		}
		 Encoder_Init_TIM2();            //=====�������ӿ�
	  MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
  while(1){
			delay_flag=1;	              //===50ms�жϾ�׼��ʱ��־λ
			oled_show();                //===��ʾ����	  	
			DataScope();			           //===��λ��
			while(delay_flag);          //===50ms�жϾ�׼��ʱ  ��Ҫ�ǲ�����ʾ��λ����Ҫ�ϸ��50ms��������   	
	}
}
