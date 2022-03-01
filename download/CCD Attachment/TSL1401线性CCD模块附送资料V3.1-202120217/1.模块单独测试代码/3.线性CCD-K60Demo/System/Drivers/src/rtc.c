/**
  ******************************************************************************
  * @file    rtc.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� ʵʱʱ������
  ******************************************************************************
  */
#include "rtc.h"

/***********************************************************************************************
 ���ܣ�RTC��ʼ��
 �βΣ�0
 ���أ�0
 ��⣺��ʼ��RTC
************************************************************************************************/
void RTC_Init(void)
{
	uint32_t i = 0;
	//����RTCģ��ʱ��
	SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;
	//��ֹ �����ж�
	RTC->IER &= ~(RTC_IER_TIIE_MASK|RTC_IER_TOIE_MASK |RTC_IER_TAIE_MASK);
	//����16pf���ݣ���������������������
	RTC->CR |= (0|RTC_CR_OSCE_MASK|RTC_CR_SC16P_MASK);
  //�Ǿ����ȶ��Ĺ�����ʱ
	for(i=0;i<0x600000;i++);
	//ʹ��RTC��ʱ
	RTC->SR |= RTC_SR_TCE_MASK;  
	RTC_ITConfig(RTC_IT_TAF,ENABLE);	
	//��TAR��λ��TSR �����ж�
	RTC->TAR = RTC->TSR;  
}


//�ж�����
void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState)
{
	//������
	assert_param(IS_RTC_IT(RTC_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(RTC_IT)
	{
		case RTC_IT_TAF:
			(ENABLE == NewState)?(RTC->IER |= RTC_IER_TAIE_MASK):(RTC->IER &= ~RTC_IER_TAIE_MASK);
			break;
		case RTC_IT_TOF:
			(ENABLE == NewState)?(RTC->IER |= RTC_IER_TOIE_MASK):(RTC->IER &= ~RTC_IER_TOIE_MASK);
			break;
		case RTC_IT_TIF:
			(ENABLE == NewState)?(RTC->IER |= RTC_IER_TIIE_MASK):(RTC->IER &= ~RTC_IER_TIIE_MASK);
			break;
		default:break;
	}
	
}

//����ж�״̬
ITStatus RTC_GetITStatus(uint16_t RTC_IT)
{
	ITStatus retval;
	//������
	assert_param(IS_RTC_IT(RTC_IT));
	
	switch(RTC_IT)
	{
		case RTC_IT_TAF:
 		 (RTC->SR & RTC_SR_TAF_MASK)?(retval = SET):(retval = RESET);
			break;
		case RTC_IT_TOF:
 		 (RTC->SR & RTC_SR_TOF_MASK)?(retval = SET):(retval = RESET);
			break;
		case RTC_IT_TIF:
 		 (RTC->SR & RTC_SR_TIF_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}


/***********************************************************************************************
 ���ܣ��ж��Ƿ�������
 �βΣ����
 ���أ�1 ���� 0 ƽ��
 ��⣺�ж��Ƿ������꺯��
			�·�   1  2  3  4  5  6  7  8  9  10 11 12
			����   31 29 31 30 31 30 31 31 30 31 30 31
			������ 31 28 31 30 31 30 31 31 30 31 30 31
************************************************************************************************/
static uint8_t RTC_IsLeapYear(uint16_t year)
{
	if(year % 4 == 0) //�����ܱ�4����
	{ 
		if(year % 100 == 0) 
		{ 
			if(year % 400 == 0)return 1;//�����00��β,��Ҫ�ܱ�400���� 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}
//����ʱ��
//�������ʱ��ת��Ϊ����
//��1970��1��1��Ϊ��׼
//1970~2099��Ϊ�Ϸ����
//����ֵ:0,�ɹ�;����:�������.
//�·����ݱ�			
static uint8_t const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�	  
//ƽ����·����ڱ�
static const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/***********************************************************************************************
 ���ܣ�����RTCʱ��
 �βΣ�RTC ����
 ���أ�������
 ��⣺
************************************************************************************************/
uint8_t RTC_SetCalander(RTC_CalanderTypeDef * RTC_CalanderStruct)
{
	uint16_t t;
	uint32_t seccount=0;
	if(RTC_CalanderStruct->Year < 1970||RTC_CalanderStruct->Year > 2099)return 1;	   
	for(t=1970;t<RTC_CalanderStruct->Year;t++)	//��������ݵ��������
	{
		if(RTC_IsLeapYear(t))seccount+=31622400;//�����������
		else seccount+=31536000;			  //ƽ���������
	}
	RTC_CalanderStruct->Month-=1;
	for(t=0;t<RTC_CalanderStruct->Month;t++)	   //��ǰ���·ݵ����������
	{
		seccount+=(uint32_t)mon_table[t]*86400;//�·����������
		if(RTC_IsLeapYear(RTC_CalanderStruct->Year)&& t == 1)seccount+=86400;//����2�·�����һ���������	   
	}
	seccount += (uint32_t)((RTC_CalanderStruct->Date)-1)*86400;//��ǰ�����ڵ���������� 
	seccount += (uint32_t)(RTC_CalanderStruct->Hour)*3600;//Сʱ������
	seccount += (uint32_t)(RTC_CalanderStruct->Minute)*60;	 //����������
	seccount += RTC_CalanderStruct->Second ;//�������Ӽ���ȥ
	RTC->SR &= ~RTC_SR_TCE_MASK;//�رռ����������ο��ֲ�k10 1024ҳ
	RTC->TSR = RTC_TSR_TSR(seccount);	
	RTC->TAR = RTC->TSR+1;
	RTC->SR |= RTC_SR_TCE_MASK;//���������������ο��ֲ�k10 1024ҳ
	return 0;
}

/***********************************************************************************************
 ���ܣ��������ռ���ʱ����
 �βΣ�������
 ���أ����ڴ���
 ��⣺���빫�����ڵõ�����(ֻ����1901-2099��)
************************************************************************************************/
static uint8_t RTC_GetWeek(uint16_t year,uint8_t month,uint8_t day)
{	
	uint16_t temp2;
	uint8_t yearH,yearL;
	yearH=year/100;	yearL=year%100; 
	// ���Ϊ21����,�������100  
	if (yearH>19)yearL+=100;
	// ����������ֻ��1900��֮���  
	temp2 = yearL+yearL/4;
	temp2 = temp2%7; 
	temp2 = temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
}			

/***********************************************************************************************
 ���ܣ����RTC����
 �βΣ�0
 ���أ�������
 ��⣺�õ���ǰ��ʱ�䣬���������calendar�ṹ������
************************************************************************************************/
#define SEC_IN_DAY  86400
void RTC_GetCalander(RTC_CalanderTypeDef * RTC_CalanderStruct)
{

	static uint16_t daycnt=0;
	uint32_t timecount = 0; 
	uint32_t temp = 0;
	uint16_t temp1 = 0;	  
	timecount = RTC->TSR;	 
	RTC_CalanderStruct->TSRValue = RTC->TSR;
 	temp = timecount/SEC_IN_DAY;   //�õ�����(��������Ӧ��)
	if(daycnt != temp)//����һ����
	{	  
		daycnt = temp;
		temp1 = 1970;	//��1970�꿪ʼ
		while(temp >= 365)
		{				 
			if(RTC_IsLeapYear(temp1))//������
			{
				if(temp>=366)temp-=366;//�����������
				else {temp1++;break;}  
			}
			else temp-=365;	  //ƽ�� 
			temp1++;  
		}   
		RTC_CalanderStruct->Year=temp1;//�õ����
		temp1=0;
		while(temp>=28)//������һ����
		{
			if(RTC_IsLeapYear(RTC_CalanderStruct->Year) && temp1 == 1)//�����ǲ�������/2�·�
			{
				if(temp >= 29)temp-=29;//�����������
				else break; 
			}
			else 
			{
				if(temp >= mon_table[temp1])temp-= mon_table[temp1];//ƽ��
				else break;
			}
			temp1++;  
		}
		RTC_CalanderStruct->Month = temp1+1;	//�õ��·�
		RTC_CalanderStruct->Date = temp+1;  	//�õ����� 
	}
	temp = timecount%86400;     		//�õ�������   	   
	RTC_CalanderStruct->Hour = temp/3600;     	//Сʱ
	RTC_CalanderStruct->Minute = (temp%3600)/60; 	//����	
	RTC_CalanderStruct->Second = (temp%3600)%60; 	//����
	RTC_CalanderStruct->Week = RTC_GetWeek(RTC_CalanderStruct->Year,RTC_CalanderStruct->Month,RTC_CalanderStruct->Date);//��ȡ����   
}	

/***********************************************************************************************
 ���ܣ�RTC���жϵ��ú���
 �βΣ�RTCx�û��ṹ��
 ���أ�0
 ��⣺��RTX���ж��е��ô˺���������RTC�û��ӿ�
************************************************************************************************/
void RTC_SecondIntProcess(void)
{
	RTC->SR &= ~RTC_SR_TCE_MASK; //�رռ�ʱ��
	RTC->TAR++;
	RTC->SR |= RTC_SR_TCE_MASK; //����������
}
