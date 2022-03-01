#include "adc.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define TSL_SI    PCout(3)   //SI  
#define TSL_CLK   PBout(3)   //CLK 
/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}
/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
 void RD_TSL(void) 
{
 u8 i=0,tslp=0;
  TSL_CLK=1;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1; 
  TSL_CLK=0;
  Dly_us();
      
  TSL_CLK=1;
  TSL_SI=0;
  Dly_us(); 
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0; 
    Dly_us();  //�����ع�ʱ��
    ADV[tslp]=(Get_Adc(4))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();
  }  
}
/**************************************************************************
�������ܣ�ACD��ʼ����ص�ѹ���
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void  Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//����ģ��ͨ����������     
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOC 
	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}		

/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


/**************************************************************************
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ ��λMV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;//��ص�ѹ
	Volt=Get_Adc(Battery_Ch)*3.3*11*100/4096;	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	return Volt;
}
/**************************************************************************
�������ܣ�����CCD��ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void  ccd_Init(void)
{    
//�ȳ�ʼ��IO��
 	RCC->APB2ENR|=1<<2;    //ʹ��PORTA��ʱ�� 
	GPIOA->CRL&=0XFFFF0FFF;//PA3 anolog���� 	 
 	 
	GPIOA->CRL&=0X0FFFF0FF;//PA2   7 
	GPIOA->CRL|=0X20000200;//Pa2   7 ������� 2MHZ   
	//ͨ��10/11����			 
	RCC->APB2ENR|=1<<9;    //ADC1ʱ��ʹ��	  
	RCC->APB2RSTR|=1<<9;   //ADC1��λ
	RCC->APB2RSTR&=~(1<<9);//��λ����	    
	RCC->CFGR&=~(3<<14);   //��Ƶ��������	
	//SYSCLK/DIV2=12M ADCʱ������Ϊ12M,ADC���ʱ�Ӳ��ܳ���14M!
	//���򽫵���ADC׼ȷ���½�! 
	RCC->CFGR|=2<<14;      	 

	ADC1->CR1&=0XF0FFFF;   //����ģʽ����
	ADC1->CR1|=0<<16;      //��������ģʽ  
	ADC1->CR1&=~(1<<8);    //��ɨ��ģʽ	  
	ADC1->CR2&=~(1<<1);    //����ת��ģʽ
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //�������ת��  
	ADC1->CR2|=1<<20;      //ʹ�����ⲿ����(SWSTART)!!!	����ʹ��һ���¼�������
	ADC1->CR2&=~(1<<11);   //�Ҷ���	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��7�Ĳ���ʱ��
	ADC1->SMPR2&=0XFFFF0FFF;//ͨ��3����ʱ�����	  
	ADC1->SMPR2|=7<<9;      //ͨ��3  239.5����,��߲���ʱ�������߾�ȷ��	 

	ADC1->CR2|=1<<0;	    //����ADת����	 
	ADC1->CR2|=1<<3;        //ʹ�ܸ�λУ׼  
	while(ADC1->CR2&1<<3);  //�ȴ�У׼���� 			 
    //��λ��������ò���Ӳ���������У׼�Ĵ�������ʼ�����λ��������� 		 
	ADC1->CR2|=1<<2;        //����ADУ׼	   
	while(ADC1->CR2&1<<2);  //�ȴ�У׼����
	delay_ms(1);
}		


/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void CCD(void)   
{  
	int i,j;
	usart1_send(0xff);			
   for(i=0; i<100;i++) 
  { 
    RD_TSL();           
   	j=0;
		for(j=0;j<128;j++)
		{
    if(ADV[j] ==0XFF)  ADV[j]--;    
    usart1_send(ADV[j]);
    }   
	}
}
