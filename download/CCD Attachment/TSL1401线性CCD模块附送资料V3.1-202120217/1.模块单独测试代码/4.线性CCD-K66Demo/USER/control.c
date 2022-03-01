#include "include.h"
#include "control.h"
#define T 0.156f
#define L 0.1445f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;
int count;
/*---------------------------------------------------------------
【函    数】PIT0_Interrupt 核心中断服务函数
【功    能】PIT0的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT1_IRQHandler()
{
  PIT_Flag_Clear(PIT1);       //清中断标志位
  Find_CCD_Zhongzhi();//读取CCD数据提取中线
}




