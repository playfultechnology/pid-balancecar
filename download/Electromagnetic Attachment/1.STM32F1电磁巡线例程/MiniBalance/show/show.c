#include "show.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
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
	
		//=============刷新=======================//
		OLED_Refresh_Gram();	

	}

