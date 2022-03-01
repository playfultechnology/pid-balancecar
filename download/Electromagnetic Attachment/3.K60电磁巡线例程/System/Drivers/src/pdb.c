/**
  ******************************************************************************
  * @file    pdb.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� �ɱ����ʱģ�� �����ļ�
  ******************************************************************************
  */
#include "pdb.h"
/***********************************************************************************************
 ���ܣ�PDB ��ʼ��
 �βΣ�PDB_InitStruct: PDB��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void PDB_Init(PDB_InitTypeDef * PDB_InitStruct)
{
	uint8_t i;
	uint32_t p;
	//������� 
	assert_param(IS_PDB_CONT_MODE(PDB_InitStruct->PDB_ContinuousMode));
	assert_param(IS_PDB_LDMOD(PDB_InitStruct->PDB_LoadMode));  
	
  //ʹ��PDBʱ��
  SIM->SCGC6 |= SIM_SCGC6_PDB_MASK ;
	//��0״̬�Ĵ���
	PDB0->SC = 0x00;
	//���ô���Դ
	PDB0->SC |= PDB_SC_TRGSEL(PDB_InitStruct->PDB_TriggerSourceSelect);
	//�������ߵ��ι���
	(PDB_InitStruct->PDB_ContinuousMode == PDB_CONT_MODE_ONESHOT)?(PDB0->SC &= ~PDB_SC_CONT_MASK):(PDB0->SC |= PDB_SC_CONT_MASK);
	//�������÷�Ƶ��
	p = ((CPUInfo.BusClock)*PDB_InitStruct->PDB_Period)/65535;
	for(i=0;i<8;i++)
	{
		if(p/(1<<i) < 40) break;
	}
	if(i > 7) i = 7;
	//���÷�Ƶ��
  PDB0->SC |= PDB_SC_MULT(PDB_MULT_40);
	PDB0->SC |= PDB_SC_PRESCALER(i);
	//����MOD�� IDLY 
	PDB0->MOD =  ((PDB_InitStruct->PDB_Period)*(CPUInfo.BusClock/1000))/(40*(1<<i));
	PDB0->IDLY = ((PDB_InitStruct->PDB_Period)*(CPUInfo.BusClock/1000))/(40*(1<<i));
	//ʹ��PDB
	PDB0->SC |= PDB_SC_PDBEN_MASK; 
	//PDB��ʼ����
	PDB0->SC |= PDB_SC_LDOK_MASK;
	//ʹ���������
	PDB0->SC |= PDB_SC_SWTRIG_MASK;
	
}

/***********************************************************************************************
 ���ܣ�PDB DAC������ʼ������
 �βΣ�PDB_ADC_InitStruct: PDB_ADC ��ʼ���ṹ
 ���أ�0
 ��⣺רΪPDBģ�鴥��DACģ����Ƶĳ�ʼ������  ������PDB_Init�����
************************************************************************************************/
void PDB_ADC_TriggerInit(PDB_ADC_PreTriggerInitTypeDef * PDB_ADC_InitStruct)
{
	uint16_t pt_value = 0;
	//�������
	assert_param(IS_PDB_TRIGGER_CH(PDB_ADC_InitStruct->PDB_ADC_TriggerSelect));
	assert_param(IS_PDB_ADC_PRE_TRIGGER_CHL(PDB_ADC_InitStruct->PDB_ADC_PreTriggerChl));
	assert_param(IS_FUNCTIONAL_STATE(PDB_ADC_InitStruct->PDB_ADC_Enable));
	assert_param(IS_FUNCTIONAL_STATE(PDB_ADC_InitStruct->PDB_ADC_BBEnable));
	
	pt_value = (1<<PDB_ADC_InitStruct->PDB_ADC_PreTriggerChl);
	//�������߹ر�ģ��
	if(ENABLE == PDB_ADC_InitStruct->PDB_ADC_Enable)
	{
		PDB0->CH[PDB_ADC_InitStruct->PDB_ADC_TriggerSelect].C1 |= PDB_C1_TOS(pt_value)|PDB_C1_EN(pt_value);
	}
	else
	{
		PDB0->CH[PDB_ADC_InitStruct->PDB_ADC_TriggerSelect].C1 &= ~(PDB_C1_TOS(pt_value)|PDB_C1_EN(pt_value));
	}
	//�Ƿ�ʹ��BB
	if(ENABLE == PDB_ADC_InitStruct->PDB_ADC_BBEnable)
	{
		PDB0->CH[PDB_ADC_InitStruct->PDB_ADC_TriggerSelect].C1 |= PDB_C1_BB(pt_value);
	}
	else
	{
		PDB0->CH[PDB_ADC_InitStruct->PDB_ADC_TriggerSelect].C1 &= ~PDB_C1_BB(pt_value);
	}
	//DLY
	PDB0->CH[PDB_ADC_InitStruct->PDB_ADC_TriggerSelect].DLY[PDB_ADC_InitStruct->PDB_ADC_PreTriggerChl] = 0;
}

/***********************************************************************************************
 ���ܣ�PDB �ж�����
 �βΣ�PDBx : PDBģ���
       @arg PDB0 �� PDBģ��0
       PDB_IT : PDBģ���ж�Դ
       @arg PDB_IT_ERR : PDB˳������ж�
       @arg PDB_IT_IF  : PDB���������ж�
       NewState : ʹ�ܻ��߽�ֹ
       @arg ENABLE : ʹ��
       @arg DISABLE: ��ֹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void PDB_ITConfig(PDB_Type* PDBx, uint16_t PDB_IT, FunctionalState NewState)
{
	//���������
	assert_param(IS_PDB_ALL_PERIPH(PDBx));
	assert_param(IS_PDB_IT(PDB_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(PDB_IT)
	{
		case PDB_IT_ERR:
			(ENABLE == NewState)?(PDBx->SC |= PDB_SC_PDBEIE_MASK):(PDBx->SC &= ~PDB_SC_PDBEIE_MASK);
			break;
		case PDB_IT_IF:
			(ENABLE == NewState)?(PDBx->SC |= PDB_SC_PDBIE_MASK):(PDBx->SC &= ~PDB_SC_PDBIE_MASK);
			break;
		default:break;
	}
}

/***********************************************************************************************
 ���ܣ�PDB ����жϱ�־
 �βΣ�PDBx : PDBģ���
       @arg PDB0 �� PDBģ��0
       PDB_IT : PDBģ���ж�Դ
       @arg PDB_IT_ERR : PDB˳������ж�
       @arg PDB_IT_IF  : PDB���������ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
ITStatus PDB_GetITStatus(PDB_Type* PDBx, uint16_t PDB_IT)
{
	ITStatus retval;
	//���������
	assert_param(IS_PDB_ALL_PERIPH(PDBx));
	assert_param(IS_PDB_IT(PDB_IT));
	
	switch(PDB_IT)
	{
		case PDB_IT_ERR:
			
			break;
		case PDB_IT_IF:
			(PDBx->SC & PDB_SC_PDBIF_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}

/***********************************************************************************************
 ���ܣ�PDB DMA����
 �βΣ�PDBx : PDBģ���
       @arg PDB0 �� PDBģ��0
       PDB_DMAReq : PDBģ��DMA����Դ
       @arg PDB_DMAReq_IF : PDBģ���������
       NewState : ʹ�ܻ��߽�ֹ
       @arg ENABLE : ʹ��
       @arg DISABLE: ��ֹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void PDB_DMACmd(PDB_Type* PDBx, uint16_t PDB_DMAReq, FunctionalState NewState)
{
	//���������
	assert_param(IS_PDB_ALL_PERIPH(PDBx));
	assert_param(IS_PDB_DMAREQ(PDB_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(PDB_DMAReq)
	{
		case PDB_DMAReq_IF:
			(ENABLE == NewState)?(PDBx->SC |= PDB_SC_DMAEN_MASK):(PDBx->SC &= ~PDB_SC_DMAEN_MASK);
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�PDB ����жϱ�־ 
 �βΣ�PDBx : PDBģ���
       @arg PDB0 �� PDBģ��0
       PDB_IT : PDBģ���ж�Դ
       @arg PDB_IT_ERR : PDB˳������ж�
       @arg PDB_IT_IF  : PDB���������ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
void PDB_ClearITPendingBit(PDB_Type *PDBx,uint16_t PDB_IT)
{
	//�������
	assert_param(IS_PDB_ALL_PERIPH(PDBx));
	assert_param(IS_PDB_IT(PDB_IT));
	
	switch(PDB_IT)
	{
		case PDB_IT_ERR:
			
			break;
		case PDB_IT_IF:
			PDBx->SC &= ~PDB_SC_PDBIF_MASK;
			break;
		default:break;
	}
}


