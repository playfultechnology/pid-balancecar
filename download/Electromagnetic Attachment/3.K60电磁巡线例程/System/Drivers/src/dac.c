/**
  ******************************************************************************
  * @file    dac.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� DACģ������
  ******************************************************************************
  */
#include "DAC.h"
/***********************************************************************************************
 ���ܣ���ʼ���ṹ�� ����Ĭ�ϲ���
 �βΣ�DAC_InitStruct: ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
{
	DAC_InitStruct->DAC_TrigerMode = DAC_TRIGER_MODE_SOFTWARE;
	DAC_InitStruct->DAC_BufferMode = BUFFER_MODE_NORMAL;
	DAC_InitStruct->DAC_BufferStartPostion = 0;
	DAC_InitStruct->DAC_BufferUpperLimit = 15;
	DAC_InitStruct->DAC_WaterMarkMode = WATER_MODE_4WORD;
}
/***********************************************************************************************
 ���ܣ�DAC��ʼ��
 �βΣ�DAC_InitStruct: DAC��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct)
{
	//�������
	assert_param(IS_DAC_TRIGGER_MODE(DAC_InitStruct->DAC_TrigerMode));
	assert_param(IS_DAC_BUFFER_MODE(DAC_InitStruct->DAC_BufferMode));
	assert_param(IS_DAC_WATERMARK_MODE(DAC_InitStruct->DAC_WaterMarkMode));
	
	//����DACģ��ʱ��
	SIM->SCGC2|=SIM_SCGC2_DAC0_MASK;  
	//����BUFFERģʽ
	switch(DAC_InitStruct->DAC_BufferMode)
	{
		case DAC_TRIGER_MODE_NONE:
			DAC0->C0 &= ~DAC_C0_DACTRGSEL_MASK;
			DAC0->C0 |= DAC_C0_DACSWTRG_MASK;
			break;
		case DAC_TRIGER_MODE_SOFTWARE:
			DAC0->C0 |= DAC_C0_DACTRGSEL_MASK;
			DAC0->C0 |= DAC_C0_DACSWTRG_MASK;
			break;
		case DAC_TRIGER_MODE_HARDWARE:
			DAC0->C0 &= ~DAC_C0_DACTRGSEL_MASK;
			DAC0->C0 &= ~DAC_C0_DACSWTRG_MASK;
			break;
		default:break;
	}
	//ѡ��ο�Դ2
	DAC0->C0 |= DAC_C0_DACRFS_MASK;  
	//����DACģ��
	DAC0->C0 |= DAC_C0_DACEN_MASK ;
	
	//����DAC_C1�Ĵ��� ����BUFFERģʽ
	switch(DAC_InitStruct->DAC_BufferMode)
	{
		case BUFFER_MODE_NORMAL:
			DAC0->C1 |= DAC_C1_DACBFEN_MASK;
			DAC0->C1 |= DAC_C1_DACBFMD(0);
			break;
		case BUFFER_MODE_SWING:
			DAC0->C1 |= DAC_C1_DACBFEN_MASK;
			DAC0->C1 |= DAC_C1_DACBFMD(1);  
			break;
		case BUFFER_MODE_ONETIMESCAN:
			DAC0->C1 |= DAC_C1_DACBFEN_MASK;
			DAC0->C1 |= DAC_C1_DACBFMD(2);
			break;
		case BUFFER_MODE_DISABLE:
			DAC0->C1 &= ~DAC_C1_DACBFEN_MASK;
			break;
	}
	//����ˮλ
	switch(DAC_InitStruct->DAC_WaterMarkMode)
	{
		case WATER_MODE_1WORD:
			DAC0->C1 |= DAC_C1_DACBFWM(0);
			break;
		case WATER_MODE_2WORD:
			DAC0->C1 |= DAC_C1_DACBFWM(1);
			break;
		case WATER_MODE_3WORD:
			DAC0->C1 |= DAC_C1_DACBFWM(2);
			break;
		case WATER_MODE_4WORD:
			DAC0->C1 |= DAC_C1_DACBFWM(3);
			break;
		default:break;
	}

  //����C2�Ĵ��� �������޺�����
	DAC0->C2 =  DAC_C2_DACBFUP(DAC_InitStruct->DAC_BufferUpperLimit);
	DAC0->C2 |= DAC_C2_DACBFRP(DAC_InitStruct->DAC_BufferStartPostion);
}

/***********************************************************************************************
 ���ܣ�ʹ��DAC��DMAģ��
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
       DAC_DMAReq : DAC DMA����Դ
       @arg DAC_DMAReq_DAC: DAC��������־��λ
       NewState : ʹ�ܻ��߹ر�
       @arg ENABLE : ʹ��
       @arg DISABLE: �ر�
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_DMACmd(DAC_Type* DACx, uint16_t DAC_DMAReq, FunctionalState NewState)
{
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	assert_param(IS_DAC_DMAREQ(DAC_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	switch(DAC_DMAReq)
	{
		case DAC_DMAReq_DAC :
			(ENABLE == NewState)?(DACx->C1 |= DAC_C1_DMAEN_MASK):(DACx->C1 &= ~DAC_C1_DMAEN_MASK);
			break;
			default:break;
	}
}

/***********************************************************************************************
 ���ܣ�DAC �ж�����
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
       DAC_IT : DAC�ж�Դ
			 @arg DAC_IT_POINTER_BUTTOM: POINTER����ײ�ʱ����
			 @arg DAC_IT_POINTER_TOP:    POINTER���ﶥ��ʱ���� 
			 @arg DAC_IT_WATER_MARK:     POINTER����ˮλʱ����
       NewState : ʹ�ܻ��߹ر�
       @arg ENABLE : ʹ��
       @arg DISABLE: �ر�
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_ITConfig(DAC_Type* DACx, uint16_t DAC_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	assert_param(IS_DAC_IT(DAC_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	switch(DAC_IT)
	{
		case DAC_IT_POINTER_BUTTOM:
			(ENABLE == NewState)?(DAC0->C0 |= DAC_C0_DACBBIEN_MASK):(DAC0->C0 &= ~DAC_C0_DACBBIEN_MASK);
			break;
		case DAC_IT_POINTER_TOP:
			(ENABLE == NewState)?(DAC0->C0 |= DAC_C0_DACBTIEN_MASK):(DAC0->C0 &= ~DAC_C0_DACBTIEN_MASK);
			break;
		case DAC_IT_WATER_MARK:
			(ENABLE == NewState)?(DAC0->C0 |= DAC_C0_DACBWIEN_MASK):(DAC0->C0 &= ~DAC_C0_DACBWIEN_MASK);
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�DAC ����жϱ�־״̬
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
       DAC_IT : DAC�ж�Դ
			 @arg DAC_IT_POINTER_BUTTOM: POINTER����ײ�ʱ����
			 @arg DAC_IT_POINTER_TOP:    POINTER���ﶥ��ʱ���� 
			 @arg DAC_IT_WATER_MARK:     POINTER����ˮλʱ����
 ���أ�SET or RESET
 ��⣺0
************************************************************************************************/
ITStatus DAC_GetITStatus(DAC_Type* DACx, uint16_t DAC_IT)
{
	ITStatus retval;
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	assert_param(IS_DAC_IT(DAC_IT));
	
	switch(DAC_IT)
	{
		case DAC_IT_POINTER_BUTTOM:
		  (DACx->SR & DAC_SR_DACBFRPBF_MASK)?(retval = SET):(retval = RESET);
			break;
		case DAC_IT_POINTER_TOP:
			(DACx->SR & DAC_SR_DACBFRPTF_MASK)?(retval = SET):(retval = RESET);
			break;
		case DAC_IT_WATER_MARK:
			(DACx->SR & DAC_SR_DACBFWMF_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}

/***********************************************************************************************
 ���ܣ�DAC �������һ��
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_SoftwareTrigger(DAC_Type *DACx)
{
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	
  DAC0->C0 |= DAC_C0_DACSWTRG_MASK;//�������һ��
}

/***********************************************************************************************
 ���ܣ�DAC ����DAC������
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
			 pDACBuffer : ������ָ��
       NumberOfBuffer : ��������С <=15
 ���أ�0
 ��⣺0
************************************************************************************************/
void DAC_SetBuffer(DAC_Type *DACx, uint16_t* pDACBuffer,uint8_t NumberOfBuffer)
{
	uint8_t i;
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	assert_param(IS_DAC_BUFFER_CNT(NumberOfBuffer));
	
	for(i=0;i<NumberOfBuffer;i++)
	{
		DACx->DAT[i].DATL = (pDACBuffer[i] & 0x00FF);
		DACx->DAT[i].DATH = (pDACBuffer[i] & 0xFF00)>>8;
	}                       
}
/***********************************************************************************************
 ���ܣ��� DAC��ֹBUFFERģʽʱ ����DAC���ֵ
 �βΣ�DACx: DACģ��
       @arg DAC0 : DAC0ģ��
			 DAC_Value : ���ֵ 0-4095
 ���أ�0
 ��⣺ʵ���� ����DACBuffer[0] ��ֵ 
************************************************************************************************/
void DAC_SetValue(DAC_Type *DACx,uint16_t DAC_Value)
{
	//�������
	assert_param(IS_DAC_ALL_PERIPH(DACx));
	assert_param(IS_DAC_BUFFER_VALUE(DAC_Value));
	
	DACx->DAT[0].DATL = (DAC_Value & 0x00FF);
	DACx->DAT[0].DATH = (DAC_Value & 0xFF00)>>8;
}


