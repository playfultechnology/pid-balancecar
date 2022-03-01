#include "timer.h"

/**************************************************************************
�������ܣ���ʱ��7��ʼ������
��ڲ�����arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ�� 
�� �� ֵ����
**************************************************************************/ 
void TIM7_Init(u16 arr, u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  	//TIM7ʱ��ʹ��    
	
	/*** Initialize timer 7 || ��ʼ����ʱ��7***/
	//Set the counter to automatically reload //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 

	TIM_ITConfig(TIM7, TIM_IT_Update,	ENABLE);
	TIM_Cmd(TIM7, ENABLE);
	
  /*** interrupt packet initialization || �жϷ����ʼ�� ***/
  //TIM1 interrupts //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  //Preempt priority 0 //��ռ���ȼ�0��	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	//Level 0 from priority //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1; 
	//IRQ channels are enabled //IRQͨ����ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	NVIC_Init(&NVIC_InitStructure);   
	 		
}



