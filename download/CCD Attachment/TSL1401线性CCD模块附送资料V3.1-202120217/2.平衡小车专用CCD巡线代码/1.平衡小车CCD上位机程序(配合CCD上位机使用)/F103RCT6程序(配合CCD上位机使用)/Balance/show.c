#include "show.h"

unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
  OLED_Show_CCD();
  OLED_ShowString(00,10,"ZHONG ZHI:");
  OLED_ShowNumber(90,10, CCD_Zhongzhi,3,12);
  OLED_ShowString(00,20,"YU    ZHI:");
  OLED_ShowNumber(90,20, CCD_Yuzhi,3,12);
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
	}

///**************************************************************************
//�������ܣ�����ʾ��������λ���������� �ر���ʾ��
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void DataScope(void)
//{   
//		DataScope_Get_Channel_Data( Servo-SERVO_INIT, 1 );       //��ʾ�Ƕ� ��λ���ȣ��㣩
//		DataScope_Get_Channel_Data( Encoder_Left, 2 );         //��ʾ�����������ľ��� ��λ��CM 
//		DataScope_Get_Channel_Data( Encoder_Right, 3 );                 //��ʾ��ص�ѹ ��λ��V
////		DataScope_Get_Channel_Data( 0 , 4 );   
////		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
////		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
////		DataScope_Get_Channel_Data(0, 7 );
////		DataScope_Get_Channel_Data( 0, 8 ); 
////		DataScope_Get_Channel_Data(0, 9 );  
////		DataScope_Get_Channel_Data( 0 , 10);
//		Send_Count = DataScope_Data_Generate(3);
//		for( i = 0 ; i < Send_Count; i++) 
//		{
//		while((USART1->SR&0X40)==0);  
//		USART1->DR = DataScope_OutPut_Buffer[i]; 
//		}
//}
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}
