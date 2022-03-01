#include "motor.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
void Motor_PWM_Init(u16 arr,u16 psc)
{		 
	RCC->APB1ENR|=1<<2;       //TIM4时钟使能    
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能   
	GPIOB->CRL&=0X00FFFFFF;   //PORTB6 7  8 9推挽输出
	GPIOB->CRL|=0XBB000000;   //PORTB6 7  8 9推挽输出
	GPIOB->CRH&=0XFFFFFF00;   //PORTB6 7  8 9推挽输出
	GPIOB->CRH|=0X000000BB;   //PORTB6 7  8 9推挽输出
	TIM4->ARR=arr;//设定计数器自动重装值 
	TIM4->PSC=psc;//预分频器不分频
	TIM4->CCMR1|=6<<4;//CH1 PWM1模式	
	TIM4->CCMR1|=6<<12; //CH2 PWM1模式	
	TIM4->CCMR2|=6<<4;//CH3 PWM1模式	
	TIM4->CCMR2|=6<<12; //CH4 PWM1模式	
	
	TIM4->CCMR1|=1<<3; //CH1预装载使能	  
	TIM4->CCMR1|=1<<11;//CH2预装载使能	 
	TIM4->CCMR2|=1<<3; //CH3预装载使能	  
	TIM4->CCMR2|=1<<11;//CH4预装载使能	 
	TIM4->CCER|=1<<0;  //CH1输出使能	
	TIM4->CCER|=1<<4;  //CH2输出使能	   
	TIM4->CCER|=1<<8;  //CH3输出使能	
	TIM4->CCER|=1<<12; //CH4输出使能	   
	TIM4->CR1=0x80;  //ARPE使能 
	TIM4->CR1|=0x01;   //使能定时器 							 
} 

/**************************************************************************
函数功能：舵机PWM初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR|=1<<11;       //使能TIM1时钟    
	RCC->APB2ENR|=1<<2;        //PORTA时钟使能 
	GPIOA->CRH&=0XFFFF0FFF;    //PORTA11复用输出
	GPIOA->CRH|=0X0000B000;    //PORTA11复用输出
	TIM1->ARR=arr;            //设定计数器自动重装值 
	TIM1->PSC=psc;             //预分频器不分频
	TIM1->CCMR2|=6<<12;        //CH4 PWM1模式	
	TIM1->CCMR2|=1<<11;        //CH4预装载使能	   
	TIM1->CCER|=1<<12;         //CH4输出使能	   
  TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM1->CR1 = 0x80;           //ARPE使能 
	TIM1->CR1|=0x01;          //使能定时器1 		
  TIM1->CCR4=750;	
}
