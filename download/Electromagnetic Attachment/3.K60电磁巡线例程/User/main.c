
#include "include.h"


//u8 Flag_Way=0,Flag_Show=0,Flag_Stop=1,Flag_Next;  //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
//int Encoder_Left,Encoder_Right;                  //���ұ��������������
//int Encoder_A_EXTI,Flag_Direction;
//int Encoder_Temp;
//float Velocity,Velocity_Set,Angle,Angle_Set;
//int Motor_A,Motor_B,Servo,Target_A,Target_B;  //�������������
//int Voltage;                                //��ص�ѹ������صı���
//float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
//u8 delay_50,delay_flag; //��ʱ����
//float Velocity_KP=12,Velocity_KI=12;	       //�ٶȿ���PID����
//u8 Bluetooth_Velocity=7,APP_RX;                 //����ң���ٶȺ�APP���յ�����
//u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //����CCD FLASH���


int ELE_Sensor,Sensor_Left,Sensor_Middle,Sensor_Right;;//���Ѳ�����

//
//u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������





//������
void main(void)
{
    /* ��ʼ��PLLΪ180M */
    PLL_Init(PLL180);      //ʱ������
    NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC�ж�����
    systime.init();
    LED_Init();//LED��

    OLED_Init();
    ELE_Init();


while(1)
{
  ELE_Sensor=(int)(Get_ELE_Bias());//��ȡ���Ѳ�ߴ�����ƫ����������


  oled_show();

//  printf("      Left: %d\r",Sensor_Left);
//  printf("      Middle: %d\r",Sensor_Middle);
//  printf("      Right: %d\r",Sensor_Right);
//  printf("      Sensor: %d\r\n",ELE_Sensor);


  delay_ms(50);
  }
}

