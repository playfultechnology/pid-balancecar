/**
  ******************************************************************************
  * @file    tsi.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.6.23
  * @brief   ����K60�̼��� Ƭ��tsi �����ļ�
  ******************************************************************************
  */
#include "tsi.h"
/***********************************************************************************************
 ���ܣ�TSI ��ʼ��
 �βΣ�TSI_InitStruct: TSI ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void TSI_Init(TSI_InitTypeDef* TSI_InitStruct)
{
	TSI_Type *TSIx = TSI0;
	PORT_Type *TSI_PORT = NULL;
	TSI_MapTypeDef *pTSI_Map = (TSI_MapTypeDef*)&(TSI_InitStruct->TSIxMAP);
	//����TSIʱ��
	SIM->SCGC5 |= (SIM_SCGC5_TSI_MASK); 	
	//�ҳ�PORT�˿�
	switch(pTSI_Map->TSI_GPIO_Index)
	{
		case 0:
			TSI_PORT = PORTA;
			SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
			break;
		case 1:
			TSI_PORT = PORTB;
			SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
			break;	
		case 2:
			TSI_PORT = PORTC;
			SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
			break;
		case 3:
			TSI_PORT = PORTD;
			SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
			break;
		case 4:
			TSI_PORT = PORTE;
			SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
			break;
	}
	//���ö�Ӧ��IO��ΪPWMģʽ
	TSI_PORT->PCR[pTSI_Map->TSI_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	TSI_PORT->PCR[pTSI_Map->TSI_Pin_Index] |= PORT_PCR_MUX(pTSI_Map->TSI_Alt_Index);
	
	//ÿ���缫����ɨ��11�Σ�prescaler��Ƶ8���������
	TSIx->GENCS |= ((TSI_GENCS_NSCN(0x18))|(TSI_GENCS_PS(7))); 
	//����TSIɨ�������ģʽ
	TSIx->SCANC &= ~TSI_SCANC_EXTCHRG_MASK; //�ⲿ���������� 0-31
	TSIx->SCANC |=  TSI_SCANC_EXTCHRG(31);//
	TSIx->SCANC &= ~TSI_SCANC_REFCHRG_MASK; //�ο�ʱ�ӳ���·   0-31
	TSIx->SCANC |=  TSI_SCANC_REFCHRG(31);  //
	TSIx->SCANC &= ~TSI_SCANC_DELVOL_MASK;  //������ѹ�趨ѡ��
	TSIx->SCANC |=  TSI_SCANC_DELVOL(7);
	//���ɨ��ģʽ������ʱ��ΪBusclock/128
	TSIx->SCANC |= (TSI_SCANC_SMOD(0))|(TSI_SCANC_AMPSC(0));
	//ʱ�ܶ�Ӧ��TSI �˿� ʹ��ʱ�����Ƚ�ֹTSI
	TSIx->GENCS &= ~TSI_GENCS_TSIEN_MASK; 
	while((TSIx->GENCS & TSI_GENCS_SCNIP_MASK) == 0x01);
	TSI0->PEN |= ((1<<pTSI_Map->TSI_CH_Index));				
	TSIx->GENCS |= TSI_GENCS_TSIEN_MASK;  
	//��У׼
	TSI_SelfCalibration(pTSI_Map->TSI_CH_Index); 
	TSIx->GENCS |= TSI_GENCS_STM_MASK; //����TSIӲ��������ɨ��
	TSIx->GENCS |= TSI_GENCS_TSIEN_MASK; //ʱ��TSI
	//�ж�ģʽѡ��
	(TSI_InitStruct->TSI_ITMode == TSI_IT_MODE_END_OF_SCAN)?(TSI0->GENCS |= TSI_GENCS_ESOR_MASK):(TSI0->GENCS &= ~TSI_GENCS_ESOR_MASK);
}

/***********************************************************************************************
 ���ܣ�TSI �ж�����
 �βΣ�TSIx: TSI ģ���
       @arg TSI0 : TSI0ģ��
       TSI_IT : TSI �ж�Դ
			 @arg TSI_IT_EOSF    : ɨ������ж�
			 @arg TSI_IT_OUTRGF  : ������Χ�ж�
			 @arg TSI_IT_EXTERF  : �ⲿ��·�ж�
			 @arg TSI_IT_OVRF    : OVERRUN�ж�
       NewState : ʹ�ܻ��߽�ֹ
       @arg ENABLE:ʹ��
       @arg DISALBE:��ֹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void TSI_ITConfig(TSI_Type *TSIx,uint16_t TSI_IT,FunctionalState NewState)
{
	switch(TSI_IT)
	{
		case TSI_IT_EOSF:
		case TSI_IT_OUTRGF:
			(NewState == ENABLE)?(TSIx->GENCS |= TSI_GENCS_TSIIE_MASK):(TSIx->GENCS &= ~TSI_GENCS_TSIIE_MASK);
			break;
		case TSI_IT_EXTERF:
		case TSI_IT_OVRF:
			(NewState == ENABLE)?(TSIx->GENCS |= TSI_GENCS_ERIE_MASK):(TSIx->GENCS &= ~TSI_GENCS_ERIE_MASK);
			break;
		default:break;
	}
}

/***********************************************************************************************
 ���ܣ�TSI ����жϱ�־λ
 �βΣ�TSIx: TSI ģ���
       @arg TSI0 : TSI0ģ��
       TSI_IT : TSI �ж�Դ
			 @arg TSI_IT_EOSF    : ɨ������ж�
			 @arg TSI_IT_OUTRGF  : ������Χ�ж�
			 @arg TSI_IT_EXTERF  : �ⲿ��·�ж�
			 @arg TSI_IT_OVRF    : OVERRUN�ж�
 ���أ�SET OR RESET
 ��⣺0
************************************************************************************************/
ITStatus TSI_GetITStatus(TSI_Type* TSIx, uint16_t TSI_IT)
{
	ITStatus retval = RESET;
	switch(TSI_IT)
	{
		case TSI_IT_EOSF:
			(TSIx->GENCS & TSI_GENCS_EOSF_MASK)?(retval = SET):(retval = RESET);
			break;
		case TSI_IT_OUTRGF:
			(TSIx->GENCS & TSI_GENCS_OUTRGF_MASK)?(retval = SET):(retval = RESET);
		  break;
		case TSI_IT_EXTERF:
			(TSIx->GENCS & TSI_GENCS_EXTERF_MASK)?(retval = SET):(retval = RESET);
			break;
		case TSI_IT_OVRF:
			(TSIx->GENCS & TSI_GENCS_OVRF_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}

/***********************************************************************************************
 ���ܣ�TSI����ж�״̬
 �βΣ�TSIx: TSI ģ���
       @arg TSI0 : TSI0ģ��
       TSI_IT : TSI �ж�Դ
			 @arg TSI_IT_EOSF    : ɨ������ж�
			 @arg TSI_IT_OUTRGF  : ������Χ�ж�
			 @arg TSI_IT_EXTERF  : �ⲿ��·�ж�
			 @arg TSI_IT_OVRF    : OVERRUN�ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
void TSI_ClearITPendingBit(TSI_Type* TSIx,uint16_t TSI_IT)
{
	switch(TSI_IT)
	{
		case TSI_IT_EOSF:
			TSIx->GENCS |= TSI_GENCS_EOSF_MASK;
			break;
		case TSI_IT_OUTRGF:
			TSIx->GENCS |= TSI_GENCS_OUTRGF_MASK;
			break;
		case TSI_IT_EXTERF:
			TSIx->GENCS |= TSI_GENCS_EXTERF_MASK;
			break;
		case TSI_IT_OVRF:
			TSIx->GENCS |= TSI_GENCS_OVRF_MASK;
			break;
		default:break;
	}
}

/***********************************************************************************************
 ���ܣ�TSI���TSI CH����ֵ
 �βΣ�TSIx: TSI ģ���
       @arg TSI0 : TSI0ģ��
       TSI_Ch : TSI �ж�Դ TSI0_CH0-TSI0-CH15
 ���أ�0
 ��⣺0
************************************************************************************************/
uint32_t TSI_GetCounter(uint8_t TSI_Ch)
{
	uint32_t ret_value = 0;
	switch(TSI_Ch)
	{
		case 0: ret_value = ELECTRODE0_COUNT; break;
		case 1: ret_value = ELECTRODE1_COUNT; break;
		case 2: ret_value = ELECTRODE2_COUNT; break;
		case 3: ret_value = ELECTRODE3_COUNT; break;
		case 4: ret_value = ELECTRODE4_COUNT; break;
		case 5: ret_value = ELECTRODE5_COUNT; break;
		case 6: ret_value = ELECTRODE6_COUNT; break;
		case 7: ret_value = ELECTRODE7_COUNT; break;
		case 8: ret_value = ELECTRODE8_COUNT; break;
		case 9: ret_value = ELECTRODE9_COUNT; break;
		case 10: ret_value = ELECTRODE10_COUNT; break;
		case 11: ret_value = ELECTRODE11_COUNT; break;
		case 12: ret_value = ELECTRODE12_COUNT; break;		
		case 13: ret_value = ELECTRODE13_COUNT; break;		
		case 14: ret_value = ELECTRODE14_COUNT; break;		
		case 15: ret_value = ELECTRODE15_COUNT; break;			
		default:break;
	}
	return ret_value;
}

//TSI���������ϵ���У׼
void TSI_SelfCalibration(uint8_t TSI_Ch)
{
	uint16_t counter_temp; 
	uint16_t i;
	TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;  //����TSI
	TSI0->GENCS |= TSI_GENCS_SWTS_MASK;  /* ����TSI����ɨ�� */
	while(!(TSI0->GENCS&TSI_GENCS_EOSF_MASK)){}; /* �ȴ�ɨ����� */
	for(i=0; i<5000;i++); /* ��ʱһ��ʱ�� */
	///����Ϊ����У����̣�����Щͨ������ʵ�������
	counter_temp = 	TSI_GetCounter(TSI_Ch);
	TSI0->THRESHLD[TSI_Ch] = TSI_THRESHLD_HTHH(counter_temp + ELECTRODE_SHAKE) | TSI_THRESHLD_LTHH(counter_temp - ELECTRODE_SHAKE);
	TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK; //�Ƚ�ֹTSI
}

/***********************************************************************************************
 ���ܣ�TSI���TSI_GetChannelOutOfRangle
 �βΣ�TSIx: TSI ģ���
       @arg TSI0 : TSI0ģ��
       TSI_Ch : TSI �ж�Դ TSI0_CH0-TSI0-CH15
 ���أ�0
 ��⣺0
************************************************************************************************/
uint8_t TSI_GetChannelOutOfRangleFlag(TSI_Type *TSIx,uint8_t TSI_CH)
{
	uint16_t state = (uint16_t)(TSI0->STATUS & 0x0000FFFF);
	if(((state >> TSI_CH) & 0x01))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/***********************************************************************************************
 ���ܣ�TSI�����б�־λ
 �βΣ�TSIx: TSI ģ���
 ���أ�0
 ��⣺0
************************************************************************************************/
void TSI_ClearAllITPendingFlag(TSI_Type *TSIx)
{
	//������ֱ�־λ
	TSIx->GENCS |= TSI_GENCS_OUTRGF_MASK;	  
	TSIx->GENCS |= TSI_GENCS_EOSF_MASK;	 
	TSIx->GENCS |= TSI_GENCS_EXTERF_MASK;
	TSIx->GENCS |= TSI_GENCS_OVRF_MASK;
	TSIx->STATUS = 0xFFFFFFFF;
}

/*
static const TSI_MapTypeDef TSI_Check_Maps[] = 
{ 
    {0, 1, 0, 0, 0}, //TSI0_CH1_PA0
    {0, 2, 0, 0, 1}, //TSI0_CH2_PA1
    {0, 3, 0, 0, 2}, //TSI0_CH3_PA2
    {0, 4, 0, 0, 3}, //TSI0_CH4_PA3
    {0, 5, 0, 0, 4}, //TSI0_CH5_PA4
    {0, 0, 0, 1, 0}, //TSI0_CH0_PB0
    {0, 6, 0, 1, 1}, //TSI0_CH6_PB1
    {0, 7, 0, 1, 2}, //TSI0_CH7_PB2
    {0, 8, 0, 1, 3}, //TSI0_CH8_PB3
    {0, 9, 0, 1,16}, //TSI0_CH9_PB16
    {0,10, 0, 1,17}, //TSI0_CH10_PB17
    {0,11, 0, 1,18}, //TSI0_CH11_PB18
    {0,12, 0, 1,19}, //TSI0_CH12_PB19
    {0,13, 0, 2, 0}, //TSI0_CH13_PC0
    {0,14, 0, 2, 1}, //TSI0_CH14_PC1
    {0,15, 0, 2, 2}, //TSI0_CH15_PC2

};

void TSI_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(TSI_Check_Maps)/sizeof(TSI_MapTypeDef);i++)
	{
		value = TSI_Check_Maps[i].TSI_Index  <<0;
		value|= TSI_Check_Maps[i].TSI_CH_Index <<2;
		value|= TSI_Check_Maps[i].TSI_Alt_Index <8;
		value|= TSI_Check_Maps[i].TSI_GPIO_Index<<11;
		value|= TSI_Check_Maps[i].TSI_Pin_Index<<14;
		printf("(0x%08xU)\r\n",value);
	}
}
*/

