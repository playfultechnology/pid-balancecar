#include "encoder.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ��ⲿ�жϲɼ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	RCC->APB2ENR|=1<<0;     	//ʹ��AFIOʱ��	
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��	   	
	GPIOA->CRL&=0XFFFFFF00; 
	GPIOA->CRL|=0X00000088;//����	
	Ex_NVIC_Config(GPIO_A,0,3);		//�����ش���
	Ex_NVIC_Config(GPIO_A,1,3);		//�����ش���
	MY_NVIC_Init(1,1,EXTI0_IRQn,2);  	//��2
	MY_NVIC_Init(1,1,EXTI1_IRQn,2);  
}
/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	RCC->APB1ENR|=1<<1;     //TIM3ʱ��ʹ��
	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	GPIOA->CRL&=0X00FFFFFF; //PA6 PA7
	GPIOA->CRL|=0X44000000; //��������
	TIM3->DIER|=1<<0;   //��������ж�				
	TIM3->DIER|=1<<6;   //�������ж�
	MY_NVIC_Init(1,3,TIM3_IRQn,1);
	TIM3->PSC = 0x0;//Ԥ��Ƶ��
	TIM3->ARR = ENCODER_TIM_PERIOD;//�趨�������Զ���װֵ 
	TIM3->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM3->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM3->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM3->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM3->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM3->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM3->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}
/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM=(short)Encoder_A_EXTI;  Encoder_A_EXTI=0; break;	//  
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/**************************************************************************
�������ܣ�TIM3�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
/**************************************************************************
�������ܣ�EXTI�жϷ�����  ͨ���ⲿ�ж�ʵ�ֱ���������
��ڲ�������
����  ֵ����
**************************************************************************/
void EXTI1_IRQHandler(void)
{			
		EXTI->PR=1<<1;  //���LINE�ϵ��жϱ�־λ
	if(PAin(1)==1)
	{
	if(PAin(0)==0)  Encoder_A_EXTI++;   
	else            Encoder_A_EXTI--;
	}
	else
	{
	if(PAin(0)==0)  Encoder_A_EXTI--;
	else            Encoder_A_EXTI++;
	}		
}
void EXTI0_IRQHandler(void)  //
{
		EXTI->PR=1<<0;  //���LINE�ϵ��жϱ�־λ
		if(PAin(0)==0)
	{
	if(PAin(1)==0)  Encoder_A_EXTI++;
	else             Encoder_A_EXTI--;
	}
	else
	{
	if(PAin(1)==0)  Encoder_A_EXTI--;
	else             Encoder_A_EXTI++;
	}		
}

