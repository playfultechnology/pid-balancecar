/**
  ******************************************************************************
  * @file    spi.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� SPIģ������
  ******************************************************************************
  */
#include "spi.h"
/***********************************************************************************************
 ���ܣ�SPI �ṹ���ʼ�� ����Ĭ�ϵĲ���
 �βΣ�SPI_InitStruct SPI ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct)
{
	SPI_InitStruct->SPI_DataSize = 8;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_512;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
}
/***********************************************************************************************
 ���ܣ�SPI ��ʼ��
 �βΣ�SPI_InitStruct SPI ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void SPI_Init(SPI_InitTypeDef* SPI_InitStruct)
{
	SPI_Type *SPIx = NULL;
	PORT_Type *SPI_PORT = NULL;
	SPI_DataMapTypeDef *pSPI_DataMap = (SPI_DataMapTypeDef*)&(SPI_InitStruct->SPIxDataMap);
	SPI_CSMapTypeDef *pSPI_CSMap = (SPI_CSMapTypeDef*)&(SPI_InitStruct->SPIxPCSMap);
	
	//�������
	assert_param(IS_SPI_DATA_CHL(SPI_InitStruct->SPIxDataMap));
	assert_param(IS_SPI_PCS_CHL(SPI_InitStruct->SPIxPCSMap));
	assert_param(IS_SPI_BAUDRATE(SPI_InitStruct->SPI_BaudRatePrescaler));
	assert_param(IS_SPI_MODE(SPI_InitStruct->SPI_Mode));
	assert_param(IS_SPI_CPHA(SPI_InitStruct->SPI_CPHA));
	assert_param(IS_SPI_CPOL(SPI_InitStruct->SPI_CPOL));
	assert_param(IS_SPI_FIRSTBIT(SPI_InitStruct->SPI_FirstBit));
	
	//�ҳ�SPIģ�� ��SPIģ��ʱ��
	switch(pSPI_DataMap->SPI_Index)
	{
		case 0:
			SIM->SCGC6 |= SIM_SCGC6_DSPI0_MASK;
			SPIx = SPI0;
			break;
		case 1:
			SIM->SCGC6 |= SIM_SCGC6_SPI1_MASK;
			SPIx = SPI1;
			break;
		case 2:
			SIM->SCGC3 |= SIM_SCGC3_SPI2_MASK;
		  SPIx = SPI2;
			break;
		default:break;     
	}
	//�ҳ���Ӧ��PORT
	switch(pSPI_DataMap->SPI_GPIO_Index)
	{
		case 0:
			SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
			SPI_PORT = PORTA;
			break;
		case 1:
			SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			SPI_PORT = PORTB;
			break;
		case 2:
			SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
			SPI_PORT = PORTC;
			break;
		case 3:
			SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
			SPI_PORT = PORTD;
			break;
		case 4:
			SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			SPI_PORT = PORTE;
			break;
		default:break;
	}
	//������Ӧ������ SCK SOUT SIN
	SPI_PORT->PCR[pSPI_DataMap->SPI_SCK_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	SPI_PORT->PCR[pSPI_DataMap->SPI_SIN_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	SPI_PORT->PCR[pSPI_DataMap->SPI_SOUT_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	SPI_PORT->PCR[pSPI_DataMap->SPI_SCK_Pin_Index] |= PORT_PCR_MUX(pSPI_DataMap->SPI_Alt_Index);
	SPI_PORT->PCR[pSPI_DataMap->SPI_SIN_Pin_Index] |= PORT_PCR_MUX(pSPI_DataMap->SPI_Alt_Index);
	SPI_PORT->PCR[pSPI_DataMap->SPI_SOUT_Pin_Index] |= PORT_PCR_MUX(pSPI_DataMap->SPI_Alt_Index);
	//����PCS
	//�ҳ���Ӧ��PORT
	switch(pSPI_CSMap->SPI_GPIO_Index)
	{
		case 0:
			SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
			SPI_PORT = PORTA;
			break;
		case 1:
			SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			SPI_PORT = PORTB;
			break;
		case 2:
			SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
			SPI_PORT = PORTC;
			break;
		case 3:
			SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
			SPI_PORT = PORTD;
			break;
		case 4:
			SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			SPI_PORT = PORTE;
			break;
		default:break;
	}
	SPI_PORT->PCR[pSPI_CSMap->SPI_PCS_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	SPI_PORT->PCR[pSPI_CSMap->SPI_PCS_Pin_Index] |= PORT_PCR_MUX(pSPI_CSMap->SPI_Alt_Index);
	//��������ģʽ
	(SPI_InitStruct->SPI_Mode == SPI_Mode_Master)?(SPIx->MCR  |= SPI_MCR_MSTR_MASK):(SPIx->MCR  &= ~SPI_MCR_MSTR_MASK);
	//����SPI��ģʽ�Ĵ���
	SPIx->MCR  = 0 & (~SPI_MCR_MDIS_MASK) 
									|SPI_MCR_HALT_MASK        //��SPI����ֹͣģʽ
									|SPI_MCR_MSTR_MASK        //����SPIΪ����ģʽ
									|SPI_MCR_PCSIS_MASK       //PCSΪ�ߵ�ƽ����SPI��������ʱ��
									|SPI_MCR_CLR_TXF_MASK     //����Ҫ���MDIS�����TXF_MASK��RXF_MASK
									|SPI_MCR_CLR_RXF_MASK  
									|SPI_MCR_DIS_TXF_MASK     //Ȼ���ٽ�ֹTXD��RXD FIFO ģʽ ����SPI���ó�����ģʽ
									|SPI_MCR_DIS_RXF_MASK; 
	//���÷�Ƶ��������
	SPIx->CTAR[1] = 0| SPI_CTAR_DBR_MASK	 //����ͨ�ŵ�
									| SPI_CTAR_PCSSCK(0)
									| SPI_CTAR_PASC(0)
									| SPI_CTAR_PBR(0)
									| SPI_CTAR_CSSCK(0)
									| SPI_CTAR_FMSZ(SPI_InitStruct->SPI_DataSize -1) //�������ݴ����λ��
									| SPI_CTAR_PDT(0);                                //����Ƭѡ�ź���������ɺ����ʱֵ 
	//��Ƶ����
	SPIx->CTAR[1] |=SPI_CTAR_BR(SPI_InitStruct->SPI_BaudRatePrescaler);							 
	//ʱ����λ����
	(SPI_InitStruct->SPI_CPHA == SPI_CPHA_1Edge)?(SPIx->CTAR[1] &= ~SPI_CTAR_CPHA_MASK):(SPIx->CTAR[1] |= SPI_CTAR_CPHA_MASK);
	//ʱ�Ӽ���
	(SPI_InitStruct->SPI_CPOL == SPI_CPOL_Low)?(SPIx->CTAR[1] &= ~SPI_CTAR_CPOL_MASK):(SPIx->CTAR[1] |= SPI_CTAR_CPOL_MASK);
	//����MSB����LSD
	(SPI_InitStruct->SPI_FirstBit == SPI_FirstBit_MSB)?(SPIx->CTAR[1] &= ~SPI_CTAR_LSBFE_MASK):(SPIx->CTAR[1] |= SPI_CTAR_LSBFE_MASK);
	//���״̬
  SPIx->SR = SPI_SR_EOQF_MASK   //���н�����־ w1c  (write 1 to clear)     
            | SPI_SR_TFUF_MASK    //TX FIFO underflow flag  w1c
            | SPI_SR_TFFF_MASK    //TX FIFO fill      flag  w1c
            | SPI_SR_RFOF_MASK    //RX FIFO overflow  flag  w1c
            | SPI_SR_RFDF_MASK    //RX FIFO fill      flasg w1c (0ʱΪ��)
					  | SPI_SR_TCF_MASK;
	//��ʼ����
	 SPIx->MCR &= ~SPI_MCR_HALT_MASK;    //��ʼ���䣬���ο��ֲ�1129ҳ
}

/***********************************************************************************************
 ���ܣ�SPI ��дһ������
 �βΣ�SPICSMap SPI Ƭѡͨ������
       @arg SPI0_PCS0_PA14: SPI0ͨ�� PCS0 PA14����
			 @arg ...
			 Data: ��Ҫ���͵�����
			 PCS_State: Ƭѡ�ź�״̬
			 @arg SPI_PCS_Asserted: ���������ݺ� Ƭѡ�ź�����
			 @arg SPI_PCS_Inactive: ���������ݺ� Ƭѡ�źű��ֵ͵�ƽ
 ���أ����յ�������
 ��⣺0
************************************************************************************************/
uint16_t SPI_ReadWriteByte(uint32_t SPICSMap,uint16_t Data,uint16_t PCS_State)
{
	uint16_t temp = 0;
	SPI_Type *SPIx = NULL;
	SPI_CSMapTypeDef *pSPI_CSMap = (SPI_CSMapTypeDef*)&(SPICSMap);
	//�������
	assert_param(IS_SPI_PCS_STATE(PCS_State));
	assert_param(IS_SPI_PCS_CHL(SPICSMap));
	//�ҳ�SPI�˿�
	switch(pSPI_CSMap->SPI_Index)
	{
		case 0:
			SPIx = SPI0;
			break;
		case 1:
			SPIx = SPI1;
			break;
		case 2:
			SPIx = SPI2;
			break;
		default:break;
	}
	while((SPIx->SR & SPI_SR_TFFF_MASK) == 0){};  //�ȴ����ͻ������п�λ
	SPIx->PUSHR = (((uint32_t)(((uint32_t)(PCS_State))<<SPI_PUSHR_CONT_SHIFT))&SPI_PUSHR_CONT_MASK) //�Ƿ�����CS
							 | SPI_PUSHR_CTAS(1)      
						   | SPI_PUSHR_PCS(1<<(pSPI_CSMap->SPI_PCS_CH_Index))//ʹ���ź�
						 	 | SPI_PUSHR_TXDATA(Data); //д����
	while(!(SPIx->SR & SPI_SR_TCF_MASK)){};     //�ȴ��������
  SPIx->SR |= SPI_SR_TCF_MASK ;               //������ͻ����־λ
  //ʹ���ջ�����Ϊ��
  while((SPIx->SR & SPI_SR_RFDF_MASK) == 0){};   //RX FIFO δ���յ�������һֱ�ȴ�
  temp = (uint8_t)(SPIx->POPR & 0xFF);           //������32λ��ʽ����POPR�У�ת����ʽ 
  SPIx->SR |= SPI_SR_RFDF_MASK;                  //�����־λ
  return temp;
}

/***********************************************************************************************
 ���ܣ�SPI �ж�����
 �βΣ�SPIx SPI ģ���
       @arg: SPI0 ģ��
			 @arg: SPI1 ģ��
       @arg: SPI2 ģ��
			 SPI_IT: �жϱ�־
			 @arg  SPI_IT_EOQF: ���н�����־
       NewState : �������߹ر�
       @arg ENABLE : ����
       @arg DISABLE: �ر�
 ���أ�0
 ��⣺0
************************************************************************************************/
void SPI_ITConfig(SPI_Type* SPIx, uint16_t SPI_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_SPI_ALL_PERIPH(SPIx));
	assert_param(IS_SPI_IT(SPI_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(SPI_IT)
	{
		case SPI_IT_EOQF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_EOQF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_EOQF_RE_MASK);
			break;
		case SPI_IT_TFFF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_TFFF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_TFFF_RE_MASK);
			break;
		case SPI_IT_TCF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_TCF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_TCF_RE_MASK);
			break;
		case SPI_IT_TFUF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_TFUF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_TFUF_RE_MASK);
			break;
		case SPI_IT_RFDF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_RFDF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_RFDF_RE_MASK);
			break;
		case SPI_IT_RFOF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_RFOF_RE_MASK):(SPIx->RSER &= ~SPI_RSER_RFOF_RE_MASK);
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�SPI ����жϱ�־
 �βΣ�SPIx SPI ģ���
       @arg: SPI0 ģ��
			 @arg: SPI1 ģ��
       @arg: SPI2 ģ��
			 SPI_IT: �жϱ�־
			 @arg  SPI_IT_EOQF: ���н�����־
 ���أ�SET OR RESET
 ��⣺0
************************************************************************************************/
ITStatus SPI_GetITStatus(SPI_Type* SPIx, uint16_t SPI_IT)
{
	ITStatus retval;
	//�������
	assert_param(IS_SPI_ALL_PERIPH(SPIx));
	assert_param(IS_SPI_IT(SPI_IT));
	
	switch(SPI_IT)
	{
		case SPI_IT_EOQF:
			(SPIx->SR & SPI_SR_EOQF_MASK)?(retval = SET):(retval = RESET);
			break;
		case SPI_IT_TFFF:
			(SPIx->SR & SPI_SR_TFFF_MASK)?(retval = SET):(retval = RESET);
			break;
		case SPI_IT_TCF:
			(SPIx->SR & SPI_SR_TCF_MASK)?(retval = SET):(retval = RESET);
			break;
		case SPI_IT_TFUF:
			(SPIx->SR & SPI_SR_TFUF_MASK)?(retval = SET):(retval = RESET);
			break;
		case SPI_IT_RFDF:
			(SPIx->SR & SPI_SR_RFDF_MASK)?(retval = SET):(retval = RESET);
			break;
		case SPI_IT_RFOF:
			(SPIx->SR & SPI_SR_RFOF_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}

/***********************************************************************************************
 ���ܣ�SPI ����жϱ�־λ
 �βΣ�SPIx SPI ģ���
       @arg: SPI0 ģ��
			 @arg: SPI1 ģ��
       @arg: SPI2 ģ��
			 SPI_IT: �жϱ�־
			 @arg  SPI_IT_EOQF: ���н�����־
 ���أ�0
 ��⣺0
************************************************************************************************/
void SPI_ClearITPendingBit(SPI_Type *SPIx,uint16_t SPI_IT)
{
	//�������
	assert_param(IS_SPI_ALL_PERIPH(SPIx));
	assert_param(IS_SPI_IT(SPI_IT));
	
	switch(SPI_IT)
	{
		case SPI_IT_EOQF:
			(SPIx->SR |= SPI_SR_EOQF_MASK);
			break;
		case SPI_IT_TFFF:
			(SPIx->SR |= SPI_SR_TFFF_MASK);
			break;
		case SPI_IT_TCF:
			(SPIx->SR |= SPI_SR_TCF_MASK);
			break;
		case SPI_IT_TFUF:
			(SPIx->SR |= SPI_SR_TFUF_MASK);
			break;
		case SPI_IT_RFDF:
			(SPIx->SR |= SPI_SR_RFDF_MASK);
			break;
		case SPI_IT_RFOF:
			(SPIx->SR |= SPI_SR_RFOF_MASK);
			break;
		default:break;
	}
}

/***********************************************************************************************
 ���ܣ�SPI DMA����
 �βΣ�SPIx SPI ģ���
       @arg: SPI0 ģ��
			 @arg: SPI1 ģ��
       @arg: SPI2 ģ��
			 SPI_IT: �жϱ�־
			 @arg  SPI_DMAReq_TFFF: ���Ͷ��н�����־
       @arg  SPI_DMAReq_RFDF: ���ջ�������
 ���أ�0
 ��⣺0
************************************************************************************************/
void SPI_DMACmd(SPI_Type* SPIx, uint16_t SPI_DMAReq, FunctionalState NewState)
{
	//�������
	assert_param(IS_SPI_ALL_PERIPH(SPIx));
	assert_param(SPI_DMAREQ(SPI_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(SPI_DMAReq)
	{
		case SPI_DMAReq_TFFF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_TFFF_DIRS_MASK):(SPIx->RSER &= ~SPI_RSER_TFFF_DIRS_MASK);
			break;
		case SPI_DMAReq_RFDF:
			(ENABLE == NewState)?(SPIx->RSER |= SPI_RSER_RFDF_DIRS_MASK):(SPIx->RSER &= ~SPI_RSER_RFDF_DIRS_MASK);
			break;
		default:break;
	}
}

/*
static const SPI_CSMapTypeDef SPI_CSCheck_Maps[] = 
{ 
    {0, 2, 0, 14, 0}, //SPI0_PCS0_PA14
    {0, 2, 2,  3, 1}, //SPI0_PCS1_PC3
    {0, 2, 2,  2, 2}, //SPI0_PCS2_PC2
    {0, 2, 2,  1, 3}, //SPI0_PCS3_PC1
    {0, 2, 2,  0, 4}, //SPI0_PCS4_PC0
    {1, 2, 1, 10, 0}, //SPI1_PCS0_PB10
    {1, 2, 1,  9, 1}, //SPI1_PCS1_PB9
    {1, 2, 4,  5, 2}, //SPI1_PCS2_PE5
    {1, 2, 4,  6, 3}, //SPI1_PCS3_PE6
    {2, 2, 1, 20, 0}, //SPI2_PCS0_PB20
};

void SPI_CalCSConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(SPI_CSCheck_Maps)/sizeof(SPI_CSMapTypeDef);i++)
	{
		value = SPI_CSCheck_Maps[i].SPI_Index<<0;
		value|= SPI_CSCheck_Maps[i].SPI_Alt_Index<<2;
		value|= SPI_CSCheck_Maps[i].SPI_GPIO_Index<<5;
		value|= SPI_CSCheck_Maps[i].SPI_PCS_Pin_Index<<8;
		printf("(0x%08xU)\r\n",value);
	}
}

static const SPI_DataMapTypeDef SPI_DataCheck_Maps[] = 
{ 
    {0, 2, 0, 15, 16, 17},
    {0, 2, 2, 5 , 6 ,  7},
    {0, 2, 3, 1 , 2 ,  3},
    {1, 2, 4, 2 , 1 ,  3},
    {1, 2, 1, 11, 16, 17},
    {2, 2, 1, 21, 22, 23},
};

void SPI_CalDataConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(SPI_DataCheck_Maps)/sizeof(SPI_DataMapTypeDef);i++)
	{
		value = SPI_DataCheck_Maps[i].SPI_Index<<0;
		value|= SPI_DataCheck_Maps[i].SPI_Alt_Index<<2;
		value|= SPI_DataCheck_Maps[i].SPI_GPIO_Index<<5;
		value|= SPI_DataCheck_Maps[i].SPI_SCK_Pin_Index<<8;
		value|= SPI_DataCheck_Maps[i].SPI_SOUT_Pin_Index<<14;
		value|= SPI_DataCheck_Maps[i].SPI_SIN_Pin_Index<<20;
		printf("(0x%08xU)\r\n",value);
	}
}
*/
