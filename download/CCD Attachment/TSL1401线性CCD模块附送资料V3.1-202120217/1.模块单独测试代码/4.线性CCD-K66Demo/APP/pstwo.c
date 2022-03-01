#include "pstwo.h"

#define PS2_DELAY5  delay_us(5)
#define PS2_DELAY16 delay_us(16)

/***********PS2 IO 定义*****************/

#define DI   GPIO_PinRead(PTD1)           //手柄的反馈信号==输入模式

#define DO_H PTD11_OUT =1       //命令位高
#define DO_L PTD11_OUT =0       //命令位低

#define CS_H PTD15_OUT =1      //CS拉高
#define CS_L PTD15_OUT =0      //CS拉低

#define CLK_H PTE11_OUT =1      //时钟拉高
#define CLK_L PTE11_OUT =0      //时钟拉低

/***********PS2 IO 定义*****************/

uint8 PS2_LX=128;//左摇杆X方向模拟量数据
uint8 PS2_LY=128;//左摇杆Y方向模拟量数据
uint8 PS2_RX=128;//右摇杆X方向模拟量数据
uint8 PS2_RY=128;//右摇杆Y方向模拟量数据
uint8 PS2_KEY;   //PS2其他按键键值数据

uint8 Comd[2]={0x01,0x42};	//开始命令，请求数据
uint8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
uint16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};//按键值


void PS2_Init(void)
{
  GPIO_PinInit(PTD1, GPI, 1);   //DI==上拉输入，统一初始化置高
  GPIO_PinInit(PTD11, GPO, 1);   //DO==输出模式，统一初始化置高
  GPIO_PinInit(PTD15, GPO, 1);  //CS==输出模式，统一初始化置高
  GPIO_PinInit(PTE11, GPO, 1);   //CLK==输出模式，统一初始化置高
}

//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
  volatile uint16 ref=0x01;

  Data[1] = 0;
  for(ref=0x01;ref<0x0100;ref<<=1)
  {
    if(ref&CMD) DO_H;            //输出一位控制位
    else DO_L;
    if(DI)
    Data[1] = ref|Data[1];
    CLK_H;                        //时钟拉高
    PS2_DELAY5;
    CLK_L;
    PS2_DELAY5;
    CLK_H;

  }
  PS2_DELAY16;
}
//判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
//返回值；0，红灯模式
//        1，其他模式
uint8 PS2_RedLight(void)
{
  CS_L;
  PS2_Cmd(Comd[0]);  //开始命令
  PS2_Cmd(Comd[1]);  //请求数据
  CS_H;
  if( Data[1] == 0X73)   return 0 ;
  else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
  static uint8 byte=0;
  static uint16 ref=0x01;
  CS_L;
  PS2_Cmd(Comd[0]);  //开始命令
  PS2_Cmd(Comd[1]);  //请求数据

  for(byte=2;byte<9;byte++)//开始接受数据
  {
    for(ref=0x01;ref<0x100;ref<<=1)
    {
      CLK_H;
      PS2_DELAY5;
      CLK_L;
      PS2_DELAY5;
      CLK_H;
      if(DI)
        Data[byte] = ref|Data[byte];
    }
    PS2_DELAY16;
  }
  CS_H;
}

//对读出来的PS2的数据进行处理,只处理按键部分
//只有一个按键按下时按下为0， 未按下为1
uint8 PS2_DataKey()
{
  uint8 index;
  uint16 Handkey;	// 按键值读取，零时存储。

  PS2_ClearData();
  PS2_ReadData();
  Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
  for(index=0;index<16;index++)
  {
    if((Handkey&(1<<(MASK[index]-1)))==0)
          return index+1;
  }
  return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
uint8 PS2_AnologData(uint8 button)
{
  return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
  uint8 a;
  for(a=0;a<9;a++)
    Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);  //开始命令
  PS2_Cmd(0x42);  //请求数据
  PS2_Cmd(0X00);
  PS2_Cmd(motor1);
  PS2_Cmd(motor2);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  PS2_DELAY16;
}
//short poll
void PS2_ShortPoll(void)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);
  PS2_Cmd(0x42);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0x00);
  CS_H;
  PS2_DELAY16;
}
//进入配置
void PS2_EnterConfing(void)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);
  PS2_Cmd(0x43);
  PS2_Cmd(0X00);
  PS2_Cmd(0x01);
  PS2_Cmd(0x00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  PS2_DELAY16;
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
  CS_L;
  PS2_Cmd(0x01);
  PS2_Cmd(0x44);
  PS2_Cmd(0X00);
  PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
  PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
  //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  PS2_DELAY16;
}
//振动设置
void PS2_VibrationMode(void)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);
  PS2_Cmd(0x4D);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0X01);
  CS_H;
  PS2_DELAY16;
}
//完成并保存配置
void PS2_ExitConfing(void)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);
  PS2_Cmd(0x43);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  CS_H;
  PS2_DELAY16;
}
//手柄配置初始化
void PS2_SetInit(void)
{
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_EnterConfing();		//进入配置模式
  PS2_DELAY16;
  PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
  PS2_DELAY16;
  //PS2_VibrationMode();	//开启震动模式
  PS2_ExitConfing();		//完成并保存配置
  PS2_DELAY16;
}

/**********************读取PS2手柄键值数据**********************/
void Read_PS2(void)
{
  PS2_KEY=PS2_DataKey();
  PS2_LX=PS2_AnologData(PSS_LX);    //PS2数据采集
  PS2_LY=PS2_AnologData(PSS_LY);
  PS2_RX=PS2_AnologData(PSS_RX);
  PS2_RY=PS2_AnologData(PSS_RY);

}














