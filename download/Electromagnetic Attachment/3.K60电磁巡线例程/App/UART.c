
#include "include.h"
#include "UART.h"

uint8_t Usart3_Receive;

void uart3_init(uint32_t bound)
{

  UART_Init(UART3,bound);
  NVIC_SetPriority(UART3_RX_TX_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
  NVIC_EnableIRQ(UART3_RX_TX_IRQn);			          //ʹ��UART4_RX_TX_IRQn���ж�

}

    /*---------------------------------------------------------------
����    ����DMA_CH4_Handler
����    �ܡ�DMAͨ��4���жϷ�����
����    ������
���� �� ֵ����
��ע�����ע������Ҫ����жϱ�־λ
----------------------------------------------------------------*/
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3_S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
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

				//��������APP���Խ���ͨѶ
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ

		 if(Flag_PID==1)  //�ɼ�����
		 {
			Receive[i]=Usart3_Receive;
			i++;
		 }
		 if(Flag_PID==2)  //��������
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
							 case 0x37:  break; //Ԥ��
							 case 0x38:  break; //Ԥ��
						 }
					 }
					 Flag_PID=0;//��ر�־λ����
					 i=0;
					 j=0;
					 Data=0;
					 memset(Receive, 0, sizeof(u8)*50);//��������
		 }
    }

}
