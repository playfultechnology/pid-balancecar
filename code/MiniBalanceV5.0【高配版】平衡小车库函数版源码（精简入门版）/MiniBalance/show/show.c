#include "show.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
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
		//=============��һ����ʾС��ģʽ=======================//	
		     if(Way_Angle==1)	OLED_ShowString(0,0,"DMP");
		else if(Way_Angle==2)	OLED_ShowString(0,0,"Kalman");
		else if(Way_Angle==3)	OLED_ShowString(0,0,"Hubu");

		//=============��������ʾ������1=======================//	
		                      OLED_ShowString(00,20,"EncoLEFT");
		if( Encoder_Left<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_Left,4,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_Left,4,12);
  	//=============��������ʾ������2=======================//		
		                      OLED_ShowString(00,30,"EncoRIGHT");
		if(Encoder_Right<0)		  OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_Right,4,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,Encoder_Right,4,12);	
		//=============��������ʾ�Ƕ�=======================//
		                      OLED_ShowString(0,50,"Angle");
		if(Angle_Balance<0)		OLED_ShowNumber(45,50,Angle_Balance+360,3,12);
		else					        OLED_ShowNumber(45,50,Angle_Balance,3,12);
		
		
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
	}
