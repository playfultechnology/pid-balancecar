#include "include.h"
#include "show.h"

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{

  OLED_Show_CCD();
  OLED_ShowString(00,10,"ZHONG ZHI:");
  OLED_ShowNumber(90,10, CCD_Zhongzhi,3,12);
  OLED_ShowString(00,20,"YU    ZHI:");
  OLED_ShowNumber(90,20, CCD_Yuzhi,3,12);

  //=============刷新=======================//
  OLED_Refresh_Gram();
}

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
