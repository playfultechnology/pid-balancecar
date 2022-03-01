#include "motor.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
void Motor_PWM_Init(u16 arr,u16 psc)
{		 
	RCC->APB1ENR|=1<<2;       //TIM4ʱ��ʹ��    
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��   
	GPIOB->CRL&=0X00FFFFFF;   //PORTB6 7  8 9�������
	GPIOB->CRL|=0XBB000000;   //PORTB6 7  8 9�������
	GPIOB->CRH&=0XFFFFFF00;   //PORTB6 7  8 9�������
	GPIOB->CRH|=0X000000BB;   //PORTB6 7  8 9�������
	TIM4->ARR=arr;//�趨�������Զ���װֵ 
	TIM4->PSC=psc;//Ԥ��Ƶ������Ƶ
	TIM4->CCMR1|=6<<4;//CH1 PWM1ģʽ	
	TIM4->CCMR1|=6<<12; //CH2 PWM1ģʽ	
	TIM4->CCMR2|=6<<4;//CH3 PWM1ģʽ	
	TIM4->CCMR2|=6<<12; //CH4 PWM1ģʽ	
	
	TIM4->CCMR1|=1<<3; //CH1Ԥװ��ʹ��	  
	TIM4->CCMR1|=1<<11;//CH2Ԥװ��ʹ��	 
	TIM4->CCMR2|=1<<3; //CH3Ԥװ��ʹ��	  
	TIM4->CCMR2|=1<<11;//CH4Ԥװ��ʹ��	 
	TIM4->CCER|=1<<0;  //CH1���ʹ��	
	TIM4->CCER|=1<<4;  //CH2���ʹ��	   
	TIM4->CCER|=1<<8;  //CH3���ʹ��	
	TIM4->CCER|=1<<12; //CH4���ʹ��	   
	TIM4->CR1=0x80;  //ARPEʹ�� 
	TIM4->CR1|=0x01;   //ʹ�ܶ�ʱ�� 							 
} 

/**************************************************************************
�������ܣ����PWM��ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR|=1<<11;       //ʹ��TIM1ʱ��    
	RCC->APB2ENR|=1<<2;        //PORTAʱ��ʹ�� 
	GPIOA->CRH&=0XFFFF0FFF;    //PORTA11�������
	GPIOA->CRH|=0X0000B000;    //PORTA11�������
	TIM1->ARR=arr;            //�趨�������Զ���װֵ 
	TIM1->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM1->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	TIM1->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	   
	TIM1->CCER|=1<<12;         //CH4���ʹ��	   
  TIM1->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	TIM1->CR1 = 0x80;           //ARPEʹ�� 
	TIM1->CR1|=0x01;          //ʹ�ܶ�ʱ��1 		
  TIM1->CCR4=750;	
}
