
#include "include.h"
#include "UART.h"

uint8_t Usart3_Receive;

void uart3_init(uint32_t bound)
{

  UART_Init(UART3,bound);
  NVIC_SetPriority(UART3_RX_TX_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
  NVIC_EnableIRQ(UART3_RX_TX_IRQn);			          //使能UART4_RX_TX_IRQn的中断

}

    /*---------------------------------------------------------------
【函    数】DMA_CH4_Handler
【功    能】DMA通道4的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3_S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
      static u8 Flag_PID,i,j,Receive[50];
      static float Data;
        Usart3_Receive=UART_GetChar(UART3);
        APP_RX=Usart3_Receive;
        if(Usart3_Receive>=0x41&&Usart3_Receive<=0x48)
            Flag_Direction=Usart3_Receive-0x40;
        else 	if(Usart3_Receive<10)
            Flag_Direction=Usart3_Receive;
	else 	if(Usart3_Receive==0X5A)
            Flag_Direction=0;

				//以下是与APP调试界面通讯
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
			Receive[i]=Usart3_Receive;
			i++;
		 }
		 if(Flag_PID==2)  //分析数据
		 {
					 if(Receive[3]==0x50) 	 PID_Send=1;
					 else  if(Receive[3]==0x57) 	 Flash_Send=1;
					 else  if(Receive[1]!=0x23)
					 {
						for(j=i;j>=4;j--)
						{
						  Data+=(Receive[j-1]-48)*pow(10,i-j);
						}
						switch(Receive[1])
						 {
							 case 0x30:  Bluetooth_Velocity=(int)Data;break;
							 case 0x31:  Velocity_KP=Data;break;
							 case 0x32:  Velocity_KI=Data;break;
							 case 0x33:  break;
							 case 0x34:  break;
							 case 0x35:  break;
							 case 0x36:  break;
							 case 0x37:  break; //预留
							 case 0x38:  break; //预留
						 }
					 }
					 Flag_PID=0;//相关标志位清零
					 i=0;
					 j=0;
					 Data=0;
					 memset(Receive, 0, sizeof(u8)*50);//数组清零
		 }
    }

}
