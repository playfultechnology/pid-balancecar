#include "stm32f10x.h"
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
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
float Zhongzhi=10;
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //��ģң�ؽ��ձ���
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //����CCD���
int main(void)
  { 
		delay_init();	    	            //=====��ʱ������ʼ��	
		uart_init(128000);	            //=====���ڳ�ʼ��Ϊ
		JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
		LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	  KEY_Init();                     //=====������ʼ��
		MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
    MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
		uart2_init(9600);               //=====���ڳ�ʼ��
    Encoder_Init_TIM2();            //=====�������ӿ�
    Encoder_Init_TIM4();            //=====��ʼ��������
		Adc_Init();                     //=====adc��ʼ��
		delay_ms(500);                  //=====��ʱ
    IIC_Init();                     //=====IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��	
    DMP_Init();                     //=====��ʼ��DMP 
		ccd_Init();                     //=====����CCD��ʼ��
    OLED_Init();                    //=====OLED��ʼ��	    
 	  TIM3_Int_Init(49,7199);       //=====��ʱ�ж�
		//NRF24L01_Init();    					//=====��ʼ��NRF24L01  
		//while(NRF24L01_Check());      //=====NRF24L01ģ���Լ죬����֮��������NRF24L01ģ�飬����ż���ִ��
		CAN1_Mode_Init(1,2,3,6,0);			//=====CAN��ʼ��,������1Mbps   
    while(1)
	   {
//				if(Flash_Send==1)        //д��PID������Flash,��app���Ƹ�ָ��
//				{
//					Flash_Write();	
//					Flash_Send=0;	
//				}	
//			if(Flag_Show==0)        	  //ʹ��MiniBalance APP��OLED��ʾ��
//				{
//						APP_Show();	
						oled_show();          //===��ʾ����
//				}
//				else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
//				{
//						DataScope();          //����MiniBalance��λ��
//				}	
//				delay_flag=1;	
//				delay_50=0;
//				while(delay_flag);	     //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
	  } 
}

