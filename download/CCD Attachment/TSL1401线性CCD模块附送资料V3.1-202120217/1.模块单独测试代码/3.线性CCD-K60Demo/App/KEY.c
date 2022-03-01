#include "include.h"
#include "KEY.h"

#define KEY_PIN PTD6                    //按键KEY_user==PTD6
#define KEY_USER GPIO_PinRead(KEY_PIN)  //读取按键电平状态
volatile uint8_t key_exti_flag = 0;
/**************************************************************************
【函    数】LED_Init
【功    能】初始化按键
【参    数】无
**************************************************************************/
void KEY_Init(void)
{
   GPIO_PinInit(KEY_PIN, GPI_UP, 1);//按键，上拉输入，初始化置高
}

/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
u8 click(void)
{
  static u8 flag_key=1;//按键按松开标志
  if(flag_key&&KEY_USER==0)
  {
    flag_key=0;
    return 1;	// 按键按下
  }
  else if(1==KEY_USER) flag_key=1;
  return 0;//无按键按下
}


/**************************************************************************
函数功能：用户按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
u8 KEY_Scan(u8 time)
{
  static u8 flag_key,count_key,double_key;
  static u16 count_single,Forever_count;
  if(KEY_USER==0)  Forever_count++;//长按标志位未置1
  else         Forever_count=0;//长按标志位未置0
  if(0==KEY_USER&&0==flag_key)  flag_key=1;//按键按下，标志位flag_key置1
  if(0==count_key)//如果为第一次按下
  {
    if(flag_key==1)//如果按下
    {
      double_key++;//计数++
      count_key=1;//完成第一次按下
    }
    if(double_key==2)//如果计数为2
    {
      double_key=0;//计数清零
      count_single=0; //
      return 2;//返回值2，双击
    }
  }
  if(1==KEY_USER)flag_key=0,count_key=0;//按键松开，清除标志位，清除按下标志

  if(1==double_key)//已经按下过一次
  {
    count_single++;
    if(count_single>time&&Forever_count<time)
    {
      double_key=0;
      count_single=0;
      return 1;//单击执行的指令
    }
    if(Forever_count>time)
    {
      double_key=0;
      count_single=0;
    }
  }
  return 0;
}

/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
  static u16 Long_Press_count,Long_Press;
  if(Long_Press==0&&KEY_USER==0)  Long_Press_count++;   //长按标志位未置1
  else                            Long_Press_count=0;
  if(Long_Press_count>200)
  {
    Long_Press=1;
    Long_Press_count=0;
    return 1;
  }
  if(Long_Press==1)     //长按标志位置1
  {
    Long_Press=0;
  }
  return 0;
}
/**************************************************************************
函数功能：选择运行的模式
入口参数：无
返回  值：无
**************************************************************************/
u8 select(void)
{
  int Angle=65;
  static u8 flag=1;
  int count;
  oled_show_once();  //OLED显示
  Encoder_Temp=abs((short)(FTM_CNT_REG(FTM2_BASE_PTR)));//转动轮子编码器计数累加
  count=Encoder_Temp;
  if(count<=Angle)	Flag_Way=0;  //APP遥控模式
  else if(count>Angle&&count<=2*Angle)	 Flag_Way=1;  //PS2遥控模式
  else if(count>2*Angle&&count<=3*Angle) Flag_Way=2;	//CCD巡线模式
  else if(count>3*Angle&&count<=4*Angle) Flag_Way=3;	//电磁巡线模式
  else FTM_CNT_REG(FTM2_BASE_PTR) = 0;
  if(KEY_USER==0)Flag_Next=1;
  if(Flag_Next==1)OLED_Clear(),flag=0;  //清除OLED屏幕 程序往下运行
  return flag;
}

