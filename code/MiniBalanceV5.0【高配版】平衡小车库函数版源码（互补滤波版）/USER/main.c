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
float Zhongzhi=-4;                          //��е��ֵ
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //��ģң�ؽ��ձ���
int main(void)
{
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
  OLED_Init();                    //=====OLED��ʼ��
	TIM3_Cap_Init(0XFFFF,84-1);	    //=====��������ʼ��
	TIM8_Cap_Init(0XFFFF,168-1);	  //=====��ģң�ؽ��ճ�ʼ
	Encoder_Init_TIM2();            //=====��������ʼ��
	Encoder_Init_TIM4();            //=====��������ʼ��
	uart2_init(9600);               //=====����2��ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP 
	Adc_Init();                     //=====ģ�����ɼ���ʼ��
	//NRF24L01_Init();    					//=====��ʼ��NRF24L01  
  //while(NRF24L01_Check());      //=====NRF24L01ģ���Լ죬����֮��������NRF24L01ģ�飬����ż���ִ��
	CAN1_Mode_Init(1,7,6,3,CAN_Mode_Normal);//=====CAN��ʼ��  
	MiniBalance_PWM_Init(8400,1);   //=====PWM��ʼ��
	MiniBalance_EXTI_Init();        //=====�ⲿ�жϳ�ʼ��
  while(1){
			if(Flash_Send==1)        //д��PID������Flash,��app���Ƹ�ָ��
				{
					Flash_Write();	
					Flash_Send=0;	
				}	
			if(Flag_Show==0)        	  //ʹ��MiniBalance APP��OLED��ʾ��
				{
						APP_Show();	
						oled_show();          //===��ʾ����
				}
				else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
				{
						DataScope();          //����MiniBalance��λ��
				}	
				delay_flag=1;	
				delay_50=0;
				while(delay_flag);	     //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
	}
}
