#include "timer.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：定时器3通道3输入捕获初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
TIM_ICInitTypeDef  TIM3_ICInitStructure;
void TIM3_Cap_Init(u16 arr,u16 psc)	
{	 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能TIM3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOB时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB0 输入  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //PB1输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//初始化定时器3 TIM3	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=03 	选择输入端 IC3映射到TI1上
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC3IE捕获中断	
  TIM_Cmd(TIM3,ENABLE ); 	//使能定时器3
}
/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;
void Read_Distane(void)
{   
	 PCout(2)=1;
	 delay_us(15);  
	 PCout(2)=0;	
			if(TIM3CH3_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			Distance=TIM3CH3_CAPTURE_STA&0X3F;
			Distance*=65536;					        //溢出时间总和
			Distance+=TIM3CH3_CAPTURE_VAL;		//得到总的高电平时间
			Distance=Distance*170/1000;
		//	printf("%d \r\n",Distance);
			TIM3CH3_CAPTURE_STA=0;			//开启下一次捕获
		}				
}
/**************************************************************************
函数功能：超声波回波脉宽读取中断
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM3->SR;
	if((TIM3CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
				{
								if(tsr&0X01)//溢出
								{	    
										if(TIM3CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
										{
											if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
											{
												TIM3CH3_CAPTURE_STA|=0X80;//标记成功捕获了一次
												TIM3CH3_CAPTURE_VAL=0XFFFF;
											}else TIM3CH3_CAPTURE_STA++;
										}	 
								}
						   	if(tsr&0x08)//捕获3发生捕获事件
				    	{	
											if(TIM3CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
											{	  			
											TIM3CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
											TIM3CH3_CAPTURE_VAL=TIM3->CCR3;	//获取当前的捕获值.
											TIM3->CCER&=~(1<<9);			//CC1P=0 设置为上升沿捕获
									  	}else  								//还未开始,第一次捕获上升沿
				   	{
											TIM3CH3_CAPTURE_STA=0;			//清空
											TIM3CH3_CAPTURE_VAL=0;
											TIM3CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
											TIM3->CNT=0;					//计数器清空
											TIM3->CCER|=1<<9; 				//CC1P=1 设置为下降沿捕获
							}		    
					    	}			     	    					   
		   }
			 TIM3->SR=0;//清除中断标志位 	     
}



u8 TIM8CH1_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

u8 TIM8CH2_CAPTURE_STA = 0;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

u8 TIM8CH3_CAPTURE_STA = 0;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

u8 TIM8CH4_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;


u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;


/**************************************************************************
函数功能：航模遥控初始化函数
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM8_Cap_Init(u16 arr, u16 psc)
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure; 
  TIM_ICInitTypeDef TIM_ICInitStructure; 
  /*RCC Configuration*/ 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 	//开启了复用时钟 
  /*GPIO Configuration*/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	 //浮空输入 
  GPIO_Init(GPIOC,&GPIO_InitStructure); 
  GPIO_ResetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7); 
  /*Timebase Configuration*/ 
  TIM_InternalClockConfig(TIM8);                                  //时钟源选择内部时钟用于计时 
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;             //预分频计数器 
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //上升沿计数 
  TIM_TimeBaseInitStructure.TIM_Period = arr;                //计数周期 
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，不分频 
  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure); 
  /*PWM_Input Configuration*/ 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //输入通道1 
  TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;      //上升沿捕获 
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //输入映射选择为对应的Tl1 
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           //预分频1 
  TIM_ICInitStructure.TIM_ICFilter = 0x00;                        //滤波器:不滤波 
  TIM_ICInit(TIM8,&TIM_ICInitStructure); 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                //输入通道2
  TIM_ICInit(TIM8,&TIM_ICInitStructure); 
  /*NVIC Configuration*/ 
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn; 
  NVIC_Init(&NVIC_InitStructure); 
  /*Interrupt Configuration*/ 
  TIM_ClearITPendingBit(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2); 
  TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2,ENABLE); 
  TIM_Cmd(TIM8,ENABLE); 
}

/**************************************************************************
函数功能：航模遥控接收中断
入口参数：无
返回  值：无
**************************************************************************/
void TIM8_CC_IRQHandler(void)
{
	if ((TIM8CH1_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获	
	{
				if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) 		//捕获1发生捕获事件
				{
								TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); 		//清除中断标志位
								if (TIM8CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
								{
									TIM8CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM8);//记录下此时的定时器计数值
									if (TIM8CH1_CAPTURE_DOWNVAL < TIM8CH1_CAPTURE_UPVAL)
									{
										TIM8_T1 = 65535;
									}
									else
										TIM8_T1 = 0;
									Remoter_Ch1 = TIM8CH1_CAPTURE_DOWNVAL - TIM8CH1_CAPTURE_UPVAL
											+ TIM8_T1;		//得到总的高电平的时间
									TIM8CH1_CAPTURE_STA = 0;		//捕获标志位清零
									TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
								}
								else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
								{
									TIM8CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM8);		//获取上升沿数据
									TIM8CH1_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
									TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling);//设置为下降沿捕获
								}
				}
	}

	if ((TIM8CH2_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)		//捕获2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);		//清除中断标志位
			if (TIM8CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM8CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM8);//记录下此时的定时器计数值
				if (TIM8CH2_CAPTURE_DOWNVAL < TIM8CH2_CAPTURE_UPVAL)
				{
					TIM8_T2 = 65535;
				}
				else
					TIM8_T2 = 0;
				Remoter_Ch2 = TIM8CH2_CAPTURE_DOWNVAL - TIM8CH2_CAPTURE_UPVAL
						+ TIM8_T2;		//得到总的高电平的时间
				TIM8CH2_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM8CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM8);		//获取上升沿数据
				TIM8CH2_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

//	if ((TIM2CH3_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
//	{
//		if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)		//捕获3发生捕获事件
//		{
//			TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);		//清除中断标志位
//			if (TIM2CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//			{
//				TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2);//记录下此时的定时器计数值
//				if (TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL)
//				{
//					TIM2_T3 = 65535;
//				}
//				else
//					TIM2_T3 = 0;
//				Remoter_Ch3 = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL
//						+ TIM2_T3;		//得到总的高电平的时间
//				TIM2CH3_CAPTURE_STA = 0;		//捕获标志位清零
//				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
//			}
//			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
//			{
//				TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);		//获取上升沿数据
//				TIM2CH3_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
//				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
//			}
//		}
//	}

//	if ((TIM2CH4_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
//	{
//		if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)		//捕获4发生捕获事件
//		{
//			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);		//清除中断标志位
//			if (TIM2CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
//			{
//				TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2);//记录下此时的定时器计数值
//				if (TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL)
//				{
//					TIM2_T4 = 65535;
//				}
//				else
//					TIM2_T4 = 0;
//				Remoter_Ch4 = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL
//						+ TIM2_T4;		//得到总的高电平的时间
//				TIM2CH4_CAPTURE_STA = 0;		//捕获标志位清零
//				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
//			}
//			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
//			{
//				TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);		//获取上升沿数据
//				TIM2CH4_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
//				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);//设置为下降沿捕获
//			}
//		}
//	}
}

void TIM8_UP_IRQHandler(void) 
{ 
// 
  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);  
}


