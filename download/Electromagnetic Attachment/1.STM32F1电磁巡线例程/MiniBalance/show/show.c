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

		OLED_ShowString(00,0,"L");
		OLED_ShowNumber(10,0,Sensor_Left,4,12);
		OLED_ShowString(40,0,"M");
		OLED_ShowNumber(50,0,Sensor_Middle,4,12);
		OLED_ShowString(80,0,"R");
		OLED_ShowNumber(90,0,Sensor_Right,4,12);
	
	OLED_ShowString(0,20,"Senser:");
	
	OLED_ShowNumber(50,20,Sensor,4,12);
	
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	

	}

