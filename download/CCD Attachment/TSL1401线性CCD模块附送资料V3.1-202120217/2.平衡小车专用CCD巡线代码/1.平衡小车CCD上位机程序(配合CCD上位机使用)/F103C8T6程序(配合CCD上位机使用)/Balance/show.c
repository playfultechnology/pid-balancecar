#include "show.h"

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
  OLED_Show_CCD();
  OLED_ShowString(00,10,"ZHONG ZHI:");
  OLED_ShowNumber(90,10, CCD_Zhongzhi,3,12);
  OLED_ShowString(00,20,"YU    ZHI:");
  OLED_ShowNumber(90,20, CCD_Yuzhi,3,12);
		//=============刷新=======================//
		OLED_Refresh_Gram();	
	}

///**************************************************************************
//函数功能：虚拟示波器往上位机发送数据 关闭显示屏
//入口参数：无
//返回  值：无
//**************************************************************************/
//void DataScope(void)
//{   
//		DataScope_Get_Channel_Data( Servo-SERVO_INIT, 1 );       //显示角度 单位：度（°）
//		DataScope_Get_Channel_Data( Encoder_Left, 2 );         //显示超声波测量的距离 单位：CM 
//		DataScope_Get_Channel_Data( Encoder_Right, 3 );                 //显示电池电压 单位：V
////		DataScope_Get_Channel_Data( 0 , 4 );   
////		DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
////		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
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
