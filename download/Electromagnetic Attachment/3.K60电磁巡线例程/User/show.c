#include "include.h"
#include "show.h"

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
  OLED_ShowString(0,10,"Z");
  OLED_ShowNumber(20,10,ELE_Sensor,4,12);



  //=============ˢ��=======================//
  OLED_Refresh_Gram();
}

