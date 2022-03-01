/**
  ******************************************************************************
  * @file    dma.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� DMA����
  ******************************************************************************
  */
#include "dma.h"
//PIT����DMA��BUG ������ͻ᲻ͣ�Ĵ���
/***********************************************************************************************
 ���ܣ���ʼ���ṹ�� ����Ĭ�ϲ���
 �βΣ�DMA_InitStruct: ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void DMA_StructInit(DMA_InitTypeDef *DMA_InitStruct)
{
	//����DMAͨ�� ����
	DMA_InitStruct->Channelx = DMA_CH0;
	DMA_InitStruct->DMAAutoClose = DISABLE;
	DMA_InitStruct->EnableState = DISABLE;
	DMA_InitStruct->PeripheralDMAReq  = 0;
	//����Ŀ�ĵ�ַ�������
	DMA_InitStruct->DestBaseAddr = (uint32_t)NULL;
	DMA_InitStruct->DestDataSize = DMA_DST_8BIT;
	DMA_InitStruct->DestMajorInc = 0;
	DMA_InitStruct->DestMinorInc = 0;
	//����Դ��ַ�������
	DMA_InitStruct->SourceBaseAddr = (uint32_t)NULL;
	DMA_InitStruct->SourceDataSize = DMA_SRC_8BIT;
	DMA_InitStruct->SourceMajorInc = 0;
	DMA_InitStruct->SourceMinorInc = 0;
}

/***********************************************************************************************
 ���ܣ���ʼ��DMAģ��
 �βΣ�DMA_InitStruct: ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void DMA_Init(DMA_InitTypeDef *DMA_InitStruct)
{
	//�������
	assert_param(IS_DMA_REQ(DMA_InitStruct->PeripheralDMAReq));
	assert_param(IS_DMA_ATTR_SSIZE(DMA_InitStruct->SourceDataSize));
	assert_param(IS_DMA_ATTR_DSIZE(DMA_InitStruct->DestDataSize));
	assert_param(IS_DMA_CH(DMA_InitStruct->Channelx));
	assert_param(IS_DMA_MINOR_LOOP(DMA_InitStruct->MinorLoopLength));
	
	//��DMA0��DMAMUXʱ��Դ
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;    
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	//����DMA����Դ
	DMAMUX->CHCFG[DMA_InitStruct->Channelx] = DMAMUX_CHCFG_SOURCE(DMA_InitStruct->PeripheralDMAReq);
	//����Դ��ַ��Ϣ	
	DMA0->TCD[DMA_InitStruct->Channelx].SADDR = DMA_InitStruct->SourceBaseAddr;
	//ִ����Դ��ַ�������Ƿ���Դ��ַ�������ۼ�
	DMA0->TCD[DMA_InitStruct->Channelx].SOFF = DMA_SOFF_SOFF(DMA_InitStruct->SourceMinorInc);
	//����Դ��ַ������
	DMA0->TCD[DMA_InitStruct->Channelx].ATTR  = 0;
	DMA0->TCD[DMA_InitStruct->Channelx].ATTR |= DMA_ATTR_SSIZE(DMA_InitStruct->SourceDataSize);
	//��ѭ��������� �Ƿ����Դ��ַ
	DMA0->TCD[DMA_InitStruct->Channelx].SLAST = DMA_InitStruct->SourceMajorInc;
	
	//����Ŀ�ĵ�ַ��Ϣ
	DMA0->TCD[DMA_InitStruct->Channelx].DADDR = DMA_InitStruct->DestBaseAddr;
	//ִ����Դ��ַ�������Ƿ���Դ��ַ�������ۼ�
	DMA0->TCD[DMA_InitStruct->Channelx].DOFF = DMA_DOFF_DOFF(DMA_InitStruct->DestMinorInc);
	//����Ŀ�ĵ�ַ������
	DMA0->TCD[DMA_InitStruct->Channelx].ATTR |= DMA_ATTR_DSIZE(DMA_InitStruct->DestDataSize);
	//��ѭ��������� �Ƿ����Դ��ַ
	DMA0->TCD[DMA_InitStruct->Channelx].DLAST_SGA = DMA_InitStruct->DestMajorInc;
	
	//���ü��������� ѭ������
	//�������ݳ��� ����ÿ�εݼ� Ҳ��������ǰ��ѭ������ current major loop count
	DMA0->TCD[DMA_InitStruct->Channelx].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(DMA_InitStruct->MinorLoopLength );
	//��ʼѭ�������� ����ѭ��������Ϊ0 ʱ�� ��װ����ʼѭ����������ֵ
	DMA0->TCD[DMA_InitStruct->Channelx].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(DMA_InitStruct->MinorLoopLength);
	//����ÿһ�δ����ֽڵĸ���  ������������ʱ DMA�㽫���ݴ���RAM 
	DMA0->TCD[DMA_InitStruct->Channelx].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(DMA_InitStruct->TransferBytes);
//����DMA TCD���ƼĴ���
	DMA0->TCD[DMA_InitStruct->Channelx].CSR = 0;
	if(DMA_InitStruct->DMAAutoClose == ENABLE)
	{
		 DMA0->TCD[DMA_InitStruct->Channelx].CSR  |=DMA_CSR_DREQ_MASK; 
	}
	else
	{
		 DMA0->TCD[DMA_InitStruct->Channelx].CSR  &=(~DMA_CSR_DREQ_MASK); 
	}
	//ʹ�ܴ˼Ĵ���DMA��ʼ����
	DMA_SetEnableReq(DMA_InitStruct->Channelx,DMA_InitStruct->EnableState);
	//DMA ͨ��ʹ��
	DMAMUX->CHCFG[DMA_InitStruct->Channelx] |= DMAMUX_CHCFG_ENBL_MASK;
}

/***********************************************************************************************
 ���ܣ��������߹ر�DMA����
 �βΣ�DMAChl: DMA0_CH0 - DMA_CH15
			 EnableState: �Ƿ�������
			 @arg ENABLE : ��������
			 @arg DISABLE: �رմ���
 ���أ�0
 ��⣺0
************************************************************************************************/
void DMA_SetEnableReq(uint8_t DMAChl,FunctionalState EnableState)
{
	//������
	assert_param(IS_DMA_CH(DMAChl));
	assert_param(IS_FUNCTIONAL_STATE(EnableState));
	
	if(EnableState == ENABLE)
	{  
		DMA0->ERQ |= (1<<DMAChl);
	}
	else
	{
		DMA0->ERQ &= ~(1<<DMAChl);
	}
}

/***********************************************************************************************
 ���ܣ��ж�DMA�����Ƿ����
 �βΣ�DMAChl: DMA0_CH0 - DMA_CH15
 ���أ�
			 @arg TRUE:  �������
			 @arg FLASE: ����δ���
 ��⣺0
************************************************************************************************/
uint8_t DMA_IsComplete(uint8_t DMAChl)
{
	//������
	assert_param(IS_DMA_CH(DMAChl));
	
	if((DMA0->TCD[DMAChl].CSR & DMA_CSR_DONE_MASK))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/***********************************************************************************************
 ���ܣ�����DMA MINOR LOOP LENGTH
 �βΣ�DMAChl: DMA0_CH0 - DMA_CH15
			 DataNumber: ѭ������
 ���أ�0
 ��⣺0
************************************************************************************************/
void DMA_SetCurrDataCounter(uint8_t DMAChl,uint16_t DataNumber)
{
	//������
	assert_param(IS_DMA_CH(DMAChl));
	assert_param(IS_DMA_MINOR_LOOP(DataNumber));
	
	DMA0->TCD[DMAChl].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(DataNumber);
}

/***********************************************************************************************
 ���ܣ���� DMA MINOR LOOP LENGTH
 �βΣ�DMAChl: DMA0_CH0 - DMA_CH15
 ���أ���ǰʣ���ѭ������
 ��⣺0
************************************************************************************************/
uint16_t DMA_GetCurrDataCounter(uint8_t DMAChl)
{
	//������
	assert_param(IS_DMA_CH(DMAChl));
	return (DMA0->TCD[DMAChl].CITER_ELINKNO & DMA_CITER_ELINKNO_CITER_MASK);
}
//ʱ���ж�
void DMA_ITConfig(DMA_Type* DMAx, uint16_t DMA_IT, uint8_t DMA_CH, FunctionalState NewState)
{
	
	switch(DMA_IT)
	{
		case DMA_IT_HALF:
			(ENABLE == NewState)?( DMA0->TCD[DMA_CH].CSR  |= DMA_CSR_INTHALF_MASK):( DMA0->TCD[DMA_CH].CSR  &= ~DMA_CSR_INTHALF_MASK);
			break;
		case DMA_IT_MAJOR:
			(ENABLE == NewState)?( DMA0->TCD[DMA_CH].CSR  |= DMA_CSR_INTMAJOR_MASK):( DMA0->TCD[DMA_CH].CSR  &= ~DMA_CSR_INTMAJOR_MASK);
			break;
		default:break;
	}
}

//����жϱ�־
ITStatus DMA_GetITStatus(DMA_Type* DMAx, uint16_t DMA_IT, uint8_t DMA_CH)
{
	ITStatus retval = RESET;
	switch(DMA_IT)
	{
		case DMA_IT_HALF:
		case DMA_IT_MAJOR:
		(DMA0->TCD[DMA_CH].CSR & DMA_CSR_DONE_MASK)?(retval = SET):(retval = RESET);
		 break;
		default:break;
	}
	return retval;
}
//����ж�Pending
void DMA_ClearITPendingBit(DMA_Type* DMAx, uint16_t DMA_IT, uint8_t DMA_CH)
{
	switch(DMA_IT)
	{
		case DMA_IT_HALF:
		case DMA_IT_MAJOR:
			DMAx->INT |= (1 << DMA_CH);
			break;
		default:break;
	}
}

