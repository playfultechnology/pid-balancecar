#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
�������ܣ�����������ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOB C Dʱ��
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11|GPIO_Pin_15;//��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;//��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;//��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO

  //IO output 0to prevent motor transfer
  //IO���0����ֹ�����ת
	GPIO_ResetBits(IN1_PORTA,IN1_PIN_A);	
	GPIO_ResetBits(IN2_PORTA,IN2_PIN_A);
	GPIO_ResetBits(IN1_PORTB,IN1_PIN_B);
	GPIO_ResetBits(IN2_PORTB,IN2_PIN_B);
	GPIO_ResetBits(IN1_PORTC,IN1_PIN_C);
	GPIO_ResetBits(IN2_PORTC,IN2_PIN_C);
	GPIO_ResetBits(IN1_PORTD,IN1_PIN_D);
	GPIO_ResetBits(IN2_PORTD,IN2_PIN_D);
}

/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ����PWM���ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	  //TIM8ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTCʱ��	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PC��
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	//Up counting mode 
	//���ϼ���ģʽ  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 
	
	// Advanced timer output must be enabled
	//�߼���ʱ���������ʹ�����		
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
	//CH1 is pre-loaded and enabled
	//CH1Ԥװ��ʹ��	 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  

  // Enable the TIMX preloaded register on the ARR
  //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	
	//Enable TIM8
	//ʹ��TIM8
	TIM_Cmd(TIM8, ENABLE);  
} 

/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
�������ܣ�ʹ�ܿ������ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //KEY��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOe0
} 

/**************************************************************************
Function: Model aircraft three channel remote control pin and steering gear PWM pin initialization
Input   : ARR: Automatic reload value  PSC: clock preset frequency
Output  : none
�������ܣ���ģ��ͨ��ң�����ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{ 
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��ʱ��
	//NVIC_InitTypeDef NVIC_InitStructure;           //�ж����ȼ�
	TIM_ICInitTypeDef TIM_ICInitStructure;         //���벶��
	//TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);

	/*** Initialize timer 1 || ��ʼ����ʱ��1 ***/
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
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


	

	/*** ��ʼ��TIM1���벶�������ͨ��1 || Initialize TIM1 for the capture parameter, channel 1 ***/
	//Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  //IC1F=0000 Configure input filter //���������˲���  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** ��ʼ��TIM1���벶�������ͨ��2 || Initialize TIM1 for the capture parameter, channel 2 ***/
	//CC1S=01 Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
	//IC1F=0000 Configure input filter //���������˲���
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** ��ʼ��TIM1���벶�������ͨ��3 || Initialize TIM1 for the capture parameter, channel 3 ***/
	//Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter //���������˲���
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
}

