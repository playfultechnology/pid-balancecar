/**
  ******************************************************************************
  * @file    lptm.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� LPTM �͹��Ķ�ʱ�� ����
  ******************************************************************************
  */
#include "lptm.h"

/***********************************************************************************************
 ���ܣ�LPTMR �͹��Ķ�ʱ�� ��ʼ��
 �βΣ�LPTM_InitStruct: LPTMR��ʼ���ṹ 
 ���أ�0
 ��⣺0
************************************************************************************************/
void LPTM_Init(LPTM_InitTypeDef* LPTM_InitStruct)
{
	LPTMR_Type* LPTMx = LPTMR0;
	//�������
	assert_param(IS_LPTM_PC_MODE(LPTM_InitStruct->LPTM_Mode));
	assert_param(IS_LPTM_MAP(LPTM_InitStruct->LPTMxMap));
	
	//����LPTM��ʱ��ģ��
	SIM->SCGC5 |= SIM_SCGC5_LPTIMER_MASK; 
	//��ռĴ���
	LPTMx->CSR = 0x00; 
  LPTMx->PSR = 0x00;
  LPTMx->CMR = 0x00;
	//��ʱ����ģʽ
	if(LPTM_InitStruct->LPTM_Mode == LPTM_Mode_TC)
	{
		LPTMx->CSR &= ~LPTMR_CSR_TMS_MASK; 
		//����Ƶ ����1KHZʱ��Դ	
		LPTMx->PSR = LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK; 
		//�趨��ʼֵ
		LPTMx->CMR = LPTMR_CMR_COMPARE(LPTM_InitStruct->LPTM_InitCompareValue); 
		//����ģ��	
		LPTMx->CSR |= LPTMR_CSR_TEN_MASK;  
	}
	else //�ⲿ�������ģʽ
	{
		LPTMx->CSR |= LPTMR_CSR_TMS_MASK; 
		//�������������½��ؼ���
		switch(LPTM_InitStruct->LPTM_Mode)
		{
			case LPTM_Mode_PC_RISING:
				LPTMx->CSR &= ~LPTMR_CSR_TPP_MASK;
				break;
			case LPTM_Mode_PC_FALLING:
				LPTMx->CSR |= LPTMR_CSR_TPP_MASK;
				break;
			default:break;
		}
		switch(LPTM_InitStruct->LPTMxMap)
		{
			case LPTM_CH1_PA19:
				SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
				//����IO��ΪLPTMͨ��
				PORTA->PCR[19] &= ~(PORT_PCR_MUX_MASK);
				PORTA->PCR[19] |= PORT_PCR_MUX(6); 
				PORTA->PCR[19] |= PORT_PCR_PE_MASK; //ʹ������������
				//ѡ������ͨ��	 
				LPTMx->CSR |= LPTMR_CSR_TPS(1);
				break;
			case LPTM_CH2_PC5:
				SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
				//����IO��ΪLPTMͨ��
				PORTC->PCR[5] &= ~(PORT_PCR_MUX_MASK);
				PORTC->PCR[5] |= PORT_PCR_MUX(4);
				PORTC->PCR[5] |= PORT_PCR_PE_MASK;    //ʹ������������
				//ѡ������ͨ��	 
				LPTMx->CSR |= LPTMR_CSR_TPS(2);
				break;
		}
		//��������ģʽ
		LPTMx->CSR |= LPTMR_CSR_TFC_MASK;   
		LPTMx->PSR |= LPTMR_PSR_PBYP_MASK;  
		//����ģ��	
		LPTMx->CSR |= LPTMR_CSR_TEN_MASK;  
	}
}

/***********************************************************************************************
 ���ܣ�LPTMR ���ö�ʱ���Ƚ�ֵ
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
			 Value : ֵ 0-0xFFFF
 ���أ�0
 ��⣺0
************************************************************************************************/
void LPTM_SetCompareValue(LPTMR_Type* LPTMx, uint32_t Value)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	LPTMx->CMR = LPTMR_CMR_COMPARE(Value); 
}
/***********************************************************************************************
 ���ܣ�LPTMR ��ö�ʱ���Ƚ�ֵ
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
 ���أ�CMRֵ  0-0xFFFF
 ��⣺0
************************************************************************************************/
uint32_t LPTM_GetCompareValue(LPTMR_Type* LPTMx)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	return (uint32_t)(LPTMx->CMR & LPTMR_CMR_COMPARE_MASK);
}
/***********************************************************************************************
 ���ܣ�LPTMR ��ü�ʱ��ֵ
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
 ���أ�CNRֵ  0-0xFFFF
 ��⣺0
************************************************************************************************/
uint32_t LPTM_GetTimerCounterValue(LPTMR_Type* LPTMx)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	return (uint32_t)(LPTMx->CNR & LPTMR_CNR_COUNTER_MASK); 
}
/***********************************************************************************************
 ���ܣ�LPTMR �ж�����
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
			 LPTM_IT: �ж�Դ
			 @arg LPTM_IT_TCF : LPTM��ʱ������ж�
       NewState : ʹ��
       @arg ENABLE:  ʹ��
       @arg DISABLE: ��ֹ
 ���أ�CNRֵ  0-0xFFFF
 ��⣺0
************************************************************************************************/
void LPTM_ITConfig(LPTMR_Type* LPTMx, uint16_t LPTM_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	if(LPTM_IT == LPTM_IT_TCF)
	{
		(ENABLE == NewState)?(LPTMx->CSR |= LPTMR_CSR_TIE_MASK):(LPTMx->CSR &= ~LPTMR_CSR_TIE_MASK);
	}
}
/***********************************************************************************************
 ���ܣ�LPTMR ���LPTM�жϱ�־
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
			 LPTM_IT: �ж�Դ
			 @arg LPTM_IT_TCF : LPTM��ʱ������ж�
 ���أ�ITStatus : ��־
       @arg SET :  ��λ
       @arg RESET: ��λ
 ��⣺0
************************************************************************************************/
ITStatus LPTM_GetITStatus(LPTMR_Type* LPTMx, uint16_t LPTM_IT)
{
	ITStatus retval;
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	if(LPTM_IT == LPTM_IT_TCF)
	{
		(LPTMx->CSR & LPTMR_CSR_TCF_MASK)?(retval = SET):(retval = RESET);
	}
	return retval;
}
/***********************************************************************************************
 ���ܣ�LPTMR ���IT��־λ
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
			 LPTM_IT: �ж�Դ
			 @arg LPTM_IT_TCF : LPTM��ʱ������ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
void LPTM_ClearITPendingBit(LPTMR_Type *LPTMx, uint16_t LPTM_IT)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	if(LPTM_IT == LPTM_IT_TCF)
	{
		LPTMx->CSR |= LPTMR_CSR_TCF_MASK;
	}
}
/***********************************************************************************************
 ���ܣ�LPTM ��ʱ����ģʽ�� ��ʱ
 �βΣ�ms: 0- 65535 ���ö�ʱ��ʱ��
 ���أ�0
 ��⣺0
************************************************************************************************/
void LPTM_DelayMs(LPTMR_Type* LPTMx, uint32_t ms)
{
  //�������
	assert_param(IS_LPTM_DELAY_TIME(ms));
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	//���ü�ʱģʽ
  LPTMx->CMR = LPTMR_CMR_COMPARE(ms); 
	//������ʱ��
	LPTMx->CSR |= LPTMR_CSR_TEN_MASK;		
	while(LPTM_GetITStatus(LPTMx,LPTM_IT_TCF) == RESET) {};
	//�رն�ʱ��
	LPTMx->CSR &= ~LPTMR_CSR_TEN_MASK;		
}
/***********************************************************************************************
 ���ܣ�LPTMR ���ü�����
 �βΣ�LPTMx: LPTMRģ���
       @arg LPTMR0 : LPTMR0ģ��
 ���أ�0
 ��⣺0
************************************************************************************************/
void LPTM_ResetTimeCounter(LPTMR_Type* LPTMx)
{
	//�������
	assert_param(IS_LPTM_ALL_PERIPH(LPTMx));
	
	//����ģ��
	LPTMx->CSR &= ~(LPTMR_CSR_TEN_MASK);
	LPTMx->CSR |= LPTMR_CSR_TEN_MASK;
}


