#include "encoder.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：外部中断采集编码器
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	RCC->APB2ENR|=1<<0;     	//使能AFIO时钟	
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	
	GPIOA->CRL&=0XFFFFFF00; 
	GPIOA->CRL|=0X00000088;//输入	
	Ex_NVIC_Config(GPIO_A,0,3);		//跳变沿触发
	Ex_NVIC_Config(GPIO_A,1,3);		//跳变沿触发
	MY_NVIC_Init(1,1,EXTI0_IRQn,2);  	//组2
	MY_NVIC_Init(1,1,EXTI1_IRQn,2);  
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	RCC->APB1ENR|=1<<1;     //TIM3时钟使能
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	GPIOA->CRL&=0X00FFFFFF; //PA6 PA7
	GPIOA->CRL|=0X44000000; //浮空输入
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->DIER|=1<<6;   //允许触发中断
	MY_NVIC_Init(1,3,TIM3_IRQn,1);
	TIM3->PSC = 0x0;//预分频器
	TIM3->ARR = ENCODER_TIM_PERIOD;//设定计数器自动重装值 
	TIM3->CR1 &=~(3<<8);// 选择时钟分频：不分频
	TIM3->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
		
	TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
	TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
	TIM3->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM3->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
	TIM3->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
	TIM3->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
	TIM3->CR1 |= 0x01;    //CEN=1，使能定时器
}
/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
函数功能：TIM3中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
/**************************************************************************
函数功能：EXTI中断服务函数  通过外部中断实现编码器计数
入口参数：无
返回  值：无
**************************************************************************/
void EXTI1_IRQHandler(void)
{			
		EXTI->PR=1<<1;  //清除LINE上的中断标志位
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
		EXTI->PR=1<<0;  //清除LINE上的中断标志位
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

