#include "include.h"
#include "control.h"
#define T 0.156f
#define L 0.1445f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;
int count;
/*---------------------------------------------------------------
����    ����PIT0_Interrupt �����жϷ�����
����    �ܡ�PIT0���жϷ�����
����    ������
���� �� ֵ����
��ע�����ע������Ҫ����жϱ�־λ
----------------------------------------------------------------*/
void PIT1_IRQHandler()
{
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ
  Find_CCD_Zhongzhi();//��ȡCCD������ȡ����
}




