#include "pstwo.h"

#define PS2_DELAY5  delay_us(5)
#define PS2_DELAY16 delay_us(16)

/***********PS2 IO ����*****************/

#define DI   GPIO_PinRead(PTD1)           //�ֱ��ķ����ź�==����ģʽ

#define DO_H PTD11_OUT =1       //����λ��
#define DO_L PTD11_OUT =0       //����λ��

#define CS_H PTD15_OUT =1      //CS����
#define CS_L PTD15_OUT =0      //CS����

#define CLK_H PTE11_OUT =1      //ʱ������
#define CLK_L PTE11_OUT =0      //ʱ������

/***********PS2 IO ����*****************/

uint8 PS2_LX=128;//��ҡ��X����ģ��������
uint8 PS2_LY=128;//��ҡ��Y����ģ��������
uint8 PS2_RX=128;//��ҡ��X����ģ��������
uint8 PS2_RY=128;//��ҡ��Y����ģ��������
uint8 PS2_KEY;   //PS2����������ֵ����

uint8 Comd[2]={0x01,0x42};	//��ʼ�����������
uint8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
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
	};//����ֵ


void PS2_Init(void)
{
  GPIO_PinInit(PTD1, GPI, 1);   //DI==�������룬ͳһ��ʼ���ø�
  GPIO_PinInit(PTD11, GPO, 1);   //DO==���ģʽ��ͳһ��ʼ���ø�
  GPIO_PinInit(PTD15, GPO, 1);  //CS==���ģʽ��ͳһ��ʼ���ø�
  GPIO_PinInit(PTE11, GPO, 1);   //CLK==���ģʽ��ͳһ��ʼ���ø�
}

//���ֱ���������
void PS2_Cmd(u8 CMD)
{
  volatile uint16 ref=0x01;

  Data[1] = 0;
  for(ref=0x01;ref<0x0100;ref<<=1)
  {
    if(ref&CMD) DO_H;            //���һλ����λ
    else DO_L;
    if(DI)
    Data[1] = ref|Data[1];
    CLK_H;                        //ʱ������
    PS2_DELAY5;
    CLK_L;
    PS2_DELAY5;
    CLK_H;

  }
  PS2_DELAY16;
}
//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��0�����ģʽ
//        1������ģʽ
uint8 PS2_RedLight(void)
{
  CS_L;
  PS2_Cmd(Comd[0]);  //��ʼ����
  PS2_Cmd(Comd[1]);  //��������
  CS_H;
  if( Data[1] == 0X73)   return 0 ;
  else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
  static uint8 byte=0;
  static uint16 ref=0x01;
  CS_L;
  PS2_Cmd(Comd[0]);  //��ʼ����
  PS2_Cmd(Comd[1]);  //��������

  for(byte=2;byte<9;byte++)//��ʼ��������
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

//�Զ�������PS2�����ݽ��д���,ֻ����������
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
uint8 PS2_DataKey()
{
  uint8 index;
  uint16 Handkey;	// ����ֵ��ȡ����ʱ�洢��

  PS2_ClearData();
  PS2_ReadData();
  Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
  for(index=0;index<16;index++)
  {
    if((Handkey&(1<<(MASK[index]-1)))==0)
          return index+1;
  }
  return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
uint8 PS2_AnologData(uint8 button)
{
  return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
  uint8 a;
  for(a=0;a<9;a++)
    Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
  CS_L;
  PS2_DELAY16;
  PS2_Cmd(0x01);  //��ʼ����
  PS2_Cmd(0x42);  //��������
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
//��������
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
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
  CS_L;
  PS2_Cmd(0x01);
  PS2_Cmd(0x44);
  PS2_Cmd(0X00);
  PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
  PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
  //0xEE������������ã���ͨ��������MODE������ģʽ��
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  PS2_DELAY16;
}
//������
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
//��ɲ���������
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
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_ShortPoll();
  PS2_DELAY16;
  PS2_EnterConfing();		//��������ģʽ
  PS2_DELAY16;
  PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
  PS2_DELAY16;
  //PS2_VibrationMode();	//������ģʽ
  PS2_ExitConfing();		//��ɲ���������
  PS2_DELAY16;
}

/**********************��ȡPS2�ֱ���ֵ����**********************/
void Read_PS2(void)
{
  PS2_KEY=PS2_DataKey();
  PS2_LX=PS2_AnologData(PSS_LX);    //PS2���ݲɼ�
  PS2_LY=PS2_AnologData(PSS_LY);
  PS2_RX=PS2_AnologData(PSS_RX);
  PS2_RY=PS2_AnologData(PSS_RY);

}














