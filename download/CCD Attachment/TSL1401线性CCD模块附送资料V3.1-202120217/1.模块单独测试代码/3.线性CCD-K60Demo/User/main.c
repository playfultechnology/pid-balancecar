
#include "include.h"
u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //����CCD FLASH���


//������
void main(void)
{
    PLL_Init(PLL180);//ʱ������180M
    NVIC_SetPriorityGrouping(0x07 - 2);  //NVIC�ж�����
    systime.init();//��ʱ
    OLED_Init();//����OLED��ʾ
    CCD_Init();//CCD��ʼ��
    MyPIT_Init();//��ʱ���жϣ��ڶ�ʱ���ж����ȡCCD����

while(1)
{
  oled_show();
  delay_ms(50);
  }
}

