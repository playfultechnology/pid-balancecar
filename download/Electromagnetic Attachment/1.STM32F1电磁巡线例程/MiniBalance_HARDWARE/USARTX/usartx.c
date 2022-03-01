#include "usartx.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

/**************************ʵ�ֺ���**********************************************
*��    ��:		usart����һ���ֽ�
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}


/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USARTʱ��
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);//������ӳ��
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //C10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  	      
				u8 temp;
				static u8 count,last_data,last_last_data,Usart_ON_Count;
		    if(Usart_ON_Flag==0)
				{	
		    if(++Usart_ON_Count>10)Usart_ON_Flag=1;
				}
				temp=USART3->DR;
				Show_Data_Mb=temp;
				   if(Usart_Flag==0)
						{	
						if(last_data==0xfe&&last_last_data==0xff) 
						Usart_Flag=1,count=0;	
						}
					 if(Usart_Flag==1)
						{	
							Urxbuf[count]=temp;     
							count++;                
							if(count==8)Usart_Flag=0;
						}
						last_last_data=last_data;
						last_data=temp;
   }
return 0;	
}
void USART_TX(void)
{
        u8 Direction_A,Direction_B,Direction_C,Direction_D;
	      u16 Temp_GZ;
	           if(Encoder_A>0) Direction_A=0;
        else if(Encoder_A<0) Direction_A=2;
	      else                 Direction_A=1;
		         if(Encoder_B>0) Direction_B=0;
        else if(Encoder_B<0) Direction_B=2;
	      else                 Direction_B=1;     
		         if(Encoder_C>0) Direction_C=0;
        else if(Encoder_C<0) Direction_C=2;
	      else                 Direction_C=1;
			       if(Encoder_D>0) Direction_D=0;
        else if(Encoder_D<0) Direction_D=2;
	      else                 Direction_D=1;   
      	Temp_GZ=gyro[2]+32768;
			
				usart3_send(0xff);		
				usart3_send(0xfe);	
				usart3_send(abs(Encoder_A));		
				usart3_send(Direction_A);	
				usart3_send(abs(Encoder_B));		
				usart3_send(Direction_B);	
				usart3_send(abs(Encoder_C));		
				usart3_send(Direction_C);	
				usart3_send(abs(Encoder_D));		
				usart3_send(Direction_D);	
				usart3_send(Temp_GZ>>8);		
				usart3_send(Temp_GZ&0x00ff);	
}



