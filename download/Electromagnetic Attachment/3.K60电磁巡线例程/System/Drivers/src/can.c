/**
  ******************************************************************************
  * @file    can.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� CAN �����ļ�
  ******************************************************************************
  */
#include "can.h"
/***********************************************************************************************
 ���ܣ�CAN ���ò�����
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 baudrate: ������
			 @arg CAN_SPEED_33K
			 @arg CAN_SPEED_83K
			 @arg CAN_SPEED_50K
			 @arg CAN_SPEED_100K
			 @arg CAN_SPEED_125K
			 @arg CAN_SPEED_250K
			 @arg CAN_SPEED_500K
			 @arg CAN_SPEED_1000K			 
 ���أ�0
 ��⣺0
************************************************************************************************/
static uint8_t CAN_SetBaudrate(CAN_Type *can,uint8_t baudrate)
{
	switch(baudrate)
	{
		case CAN_SPEED_33K:
			 // 48M/120= 400k sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 120
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(2)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(119));
		  break;
		case CAN_SPEED_83K:
			 // 48M/48= 1M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 3, PSEG1 = 4, PSEG2 = 4,PRESDIV = 48
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(2)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(47));
		  break;	
		case CAN_SPEED_50K:
			 // 48M/80= 0.6M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 40
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(1)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(79));	
		break;
		case CAN_SPEED_100K:
			 // 48M/40= 1.2M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 40
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(2)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(39));	
		break;
		case CAN_SPEED_125K:
			 // 48M/32= 1.5M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 3, PSEG1 = 4, PSEG2 = 4, PRESDIV = 32
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(2)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(31));		
		break;
		case CAN_SPEED_250K:
			 // 48M/16= 3M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 16
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(1)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(15));		
		break;
		case CAN_SPEED_500K:
			 // 48M/8=6M sclock, 12Tq
			 // PROPSEG = 3, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 2, PSEG1 = 4, PSEG2 = 4, PRESDIV = 6
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(2) 
											| CAN_CTRL1_RJW(1)
											| CAN_CTRL1_PSEG1(3) 
											| CAN_CTRL1_PSEG2(3)
											| CAN_CTRL1_PRESDIV(7));		
		break;
		case CAN_SPEED_1000K:
			 // 48M/6=8M sclock
			 // PROPSEG = 4, LOM = 0x0, LBUF = 0x0, TSYNC = 0x0, SAMP = 1
			 // RJW = 1, PSEG1 = 1, PSEG2 = 2, PRESCALER = 6
			can->CTRL1 = (0 | CAN_CTRL1_PROPSEG(3) 
											| CAN_CTRL1_RJW(0)
											| CAN_CTRL1_PSEG1(0) 
											| CAN_CTRL1_PSEG2(1)
											| CAN_CTRL1_PRESDIV(5));	
		break;
		default: return 1;
	}
	return 0;
}

/***********************************************************************************************
 ���ܣ�CAN ��ʼ��
 �βΣ�CAN_InitStruct: CAN ��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void CAN_Init(CAN_InitTypeDef* CAN_InitStruct)
{
	uint8_t i = 0;
	PORT_Type *CAN_PORT = NULL;
	CAN_MapTypeDef *pCAN_Map = (CAN_MapTypeDef*)&(CAN_InitStruct->CANxMap);
  CAN_Type  *CANx = NULL;
	//�������
	assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->FilterEnable));
	assert_param(IS_CAN_MAP(CAN_InitStruct->CANxMap));
	assert_param(IS_CAN_SPEED(CAN_InitStruct->CAN_BaudRateSelect));
	//�ҳ�CANģ�� ����CANģ��ʱ��
	switch(pCAN_Map->CAN_Index )
	{
		case 0:
			SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;
			CANx = CAN0;
			break;
		case 1:
			SIM->SCGC3 |= SIM_SCGC3_FLEXCAN1_MASK;
			CANx = CAN1;
			break;
		default:break;     
	}
	//�ҳ���Ӧ��PORT
	switch(pCAN_Map->CAN_GPIO_Index)
	{
		case 0:
			SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
			CAN_PORT = PORTA;
			break;
		case 1:
			SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			CAN_PORT = PORTB;
			break;
		case 2:
			SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
			CAN_PORT = PORTC;
			break;
		case 3:
			SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
			CAN_PORT = PORTD;
			break;
		case 4:
			SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			CAN_PORT = PORTE;
			break;
		default:break;
	}
	//������Ӧ������ 
	CAN_PORT->PCR[pCAN_Map->CAN_TX_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	CAN_PORT->PCR[pCAN_Map->CAN_RX_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	CAN_PORT->PCR[pCAN_Map->CAN_TX_Pin_Index] |= PORT_PCR_MUX(pCAN_Map->CAN_Alt_Index);
	CAN_PORT->PCR[pCAN_Map->CAN_RX_Pin_Index] |= PORT_PCR_MUX(pCAN_Map->CAN_Alt_Index);
	//ѡ��ΪBusClockʱ��
	CANx->CTRL1 |=  CAN_CTRL1_CLKSRC_MASK;	
	//ʹ�ܶ���ģʽ ֻ�н��붳��ģʽ�������ò�����
	CANx->MCR|=CAN_MCR_FRZ_MASK;
	//����CANͨ��ģ�� 
	CANx->MCR &= ~CAN_MCR_MDIS_MASK;
	//�ȴ�ͨ��ģ�鸴λ����
	while((CAN_MCR_LPMACK_MASK & (CANx->MCR)));
	//�����λģ�� //�ȴ������λ���
	CANx->MCR |= CAN_MCR_SOFTRST_MASK;
	while(CAN_MCR_SOFTRST_MASK & (CANx->MCR));
	// �ȴ�ģ����붳��ģʽ
	while(!(CAN_MCR_FRZACK_MASK & (CANx->MCR)));
	for(i=0;i<16;i++)
	{
		CANx->MB[i].CS = 0x00000000;
		CANx->MB[i].ID = 0x00000000;
		CANx->MB[i].WORD0 = 0x00000000;
		CANx->MB[i].WORD1 = 0x00000000;
	}
	//����Զ����������������������ide��Զ�����󲻱Ƚϣ�
	//�������ȴӽ���fifoƥ�����������ƥ����ο��ֲ�
	CANx->CTRL2 = (0|CAN_CTRL2_TASD(22)); 
	//����IDģʽ��Ŀǰѡ�����A��ʽ
	//���չ���Ҫ������MCR�Ĵ���
	CANx->MCR |= CAN_MCR_IDAM(0); 
	//ʹ�ܸ����˲��ͽ��ܶ����ص�?
	//����ͨ��Ƶ�ʺͳ�ʼ��CTRL1  ���Լ����ģ����ô���һbite��ʱ�䣩	 |CAN_CTRL1_LPB_MASK
  CANx->MCR |= CAN_MCR_IRMQ_MASK;    
  //���ô�������
	CAN_SetBaudrate(CANx,CAN_InitStruct->CAN_BaudRateSelect);
	CANx->MCR |= CAN_MCR_IDAM(0); //����IDģʽ��Ŀǰѡ�����A��ʽ
	//�������ι���
	if(CAN_InitStruct->FilterEnable == ENABLE)
	{
		for(i = 0; i < 16 ; i++)
		{
			CANx->RXIMR[i] = 0x1FFFFFFF; //����16������id������Ĵ��� 
			CANx->RXMGMASK = 0x1FFFFFFF;
		} 	
	}
	else
	{
		for(i = 0; i < 16 ; i++)
		{
			CANx->RXIMR[i] = 0; //����16������id������Ĵ��� 
			CANx->RXMGMASK = 0;
		} 		
	}
	//ֻ���ڶ���ģʽ�²������� �������˳�����ģʽ
	CANx->MCR &= ~(CAN_MCR_HALT_MASK);
	//�ȴ�ģ���Ƴ�����ģʽ
	while((CAN_MCR_FRZACK_MASK & (CANx->MCR)));  
	//�ȴ�ͬ��
	while(((CANx->MCR)&CAN_MCR_NOTRDY_MASK));    
	//���ý���ID���˹���
}

/***********************************************************************************************
 ���ܣ�CAN ʹ��������չ���
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 RxMessage : ������Ϣ�ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void CAN_EnableReceiveMB(CAN_Type* CANx,CAN_RxMsgTypeDef* RxMessage)
{
	//�������
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_MB_NUM(RxMessage->MBIndex));

	//����������
	CANx->MB[RxMessage->MBIndex].CS = CAN_CS_CODE(0);
	//д��ID
	//��չ֡������ͨ֡
	if(RxMessage->IDE == CAN_IDE_Extended)
	{
		CANx->MB[RxMessage->MBIndex].ID = RxMessage->Id;
		CANx->MB[RxMessage->MBIndex].CS = (0|CAN_CS_CODE(4)|CAN_CS_IDE_MASK);	 // ������� MB ��Ϊ��������
	}
	else
	{
		CANx->MB[RxMessage->MBIndex].ID = RxMessage->Id<<18; 
		CANx->MB[RxMessage->MBIndex].CS = CAN_CS_CODE(4); 
	}
}

/***********************************************************************************************
 ���ܣ�CAN ����һ����Ϣ
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 TxMessage : ������Ϣ�ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
uint8_t CAN_Transmit(CAN_Type* CANx, CAN_TxMsgTypeDef* TxMessage)
{
	uint32_t temp_id = 0;
	uint16_t i,j;
	uint32_t word[2] = {0};
	//�������
	assert_param(IS_CAN_MB_NUM(TxMessage->MBIndex));
	assert_param(IS_CAN_RTR(TxMessage->RTR));
	assert_param(IS_CAN_IDE(TxMessage->IDE));
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_DLC(TxMessage->DLC));
	assert_param(IS_CAN_IDE(TxMessage->IDE ));

	//ת�����ݸ�ʽ
	for(i=0;i<TxMessage->DLC;i++)
	{
		if(i < 4)
		{
			word[0] |= ((TxMessage->Data[i])<<((3-i)*8));
		}
		else
		{
			word[1] |= ((TxMessage->Data[i])<<((7-i)*8));		
		}
	}
	//��չ֡������ͨ֡
	(TxMessage->IDE == CAN_IDE_Extended)?(temp_id = ((1<<29)|TxMessage->Id)):(temp_id = ((1<<29)|(TxMessage->Id << 18)));
	//���䴦��
	CANx->MB[TxMessage->MBIndex].CS = CAN_CS_CODE(8); // д�Ǽ������
	CANx->MB[TxMessage->MBIndex].ID = temp_id;    
	CANx->MB[TxMessage->MBIndex].WORD0 = word[0];
	CANx->MB[TxMessage->MBIndex].WORD1 = word[1];  
	for(i = 0;i < 50;i++);	   //��ʱ������Ӳ��׼����
	if(TxMessage->IDE == 1)
	{
		CANx->MB[TxMessage->MBIndex].CS = CAN_CS_CODE(12)|CAN_CS_IDE_MASK|CAN_CS_DLC(TxMessage->DLC)|CAN_CS_SRR_MASK;
	}
	else
	{
		CANx->MB[TxMessage->MBIndex].CS = CAN_CS_CODE(12)|CAN_CS_DLC(TxMessage->DLC);
	}
	//Զ��֡��������֡
	(TxMessage->RTR == CAN_RTR_Remote)?(CANx->MB[TxMessage->MBIndex].CS |= CAN_CS_RTR_MASK):(CANx->MB[TxMessage->MBIndex].CS &= ~CAN_CS_RTR_MASK);
	j=0; 
	//�ȴ����ݷ�����ɻ���ʱ
	while(!(CANx->IFLAG1 & (1<<TxMessage->MBIndex)))
	{
		if((j++)>0x1000)
		return FALSE;
	}
	//�屨�Ļ������жϱ�־
	CANx->IFLAG1 = (1<<TxMessage->MBIndex);	 //�������
	return TRUE;
}

/***********************************************************************************************
 ���ܣ�CAN ����һ����Ϣ
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 RxMessage   : CAN������սṹ
 ���أ�
			 @arg TRUE  : ���ճɹ�
			 @arg FALSE : ����ʧ��
 ��⣺0
************************************************************************************************/
uint8_t CAN_Receive(CAN_Type* CANx,CAN_RxMsgTypeDef* RxMessage)
{
	uint8_t code = 0;
	uint8_t i = 0;
	uint8_t len = 0;
	uint32_t word[2] = {0};
	//�������
	assert_param(IS_CAN_MB_NUM(RxMessage->MBIndex));
	assert_param(IS_CAN_ALL_PERIPH(CANx));


	code = CANx->TIMER;    // ȫ�ֽ��� MB ����
	
	//�鿴��־λ
	if((CANx->IFLAG1 & (1<<(RxMessage->MBIndex))) == 0)
	{
		return FALSE;
	}
	code = CAN_get_code(CANx->MB[RxMessage->MBIndex].CS);
	if(code != 0x02)
	{
		//����ʧ��
		RxMessage->IDE = 0;
		return FALSE;
	}
	len = CAN_get_length(CANx->MB[RxMessage->MBIndex].CS);
	if(len < 1)
	{
		RxMessage->IDE = 0;
		return FALSE;
	}
	RxMessage->IDE = len;
	code = CANx->TIMER;    // ȫ�ֽ��� MB ����
	CANx->IFLAG1 = (1<<(RxMessage->MBIndex));//�������
	word[0] = CANx->MB[RxMessage->MBIndex].WORD0;   //��ȡ���յ�����
	word[1] = CANx->MB[RxMessage->MBIndex].WORD1;   //��ȡ���յ�����
	//�ж��Ǳ�׼֡������չ֡
	if(CANx->MB[RxMessage->MBIndex].CS & CAN_CS_IDE_MASK)
	{
		RxMessage->IDE = CAN_IDE_Extended;
		RxMessage->Id =  CANx->MB[RxMessage->MBIndex].ID;
	}
	else
	{
		RxMessage->IDE = CAN_IDE_Standard;
		RxMessage->Id =  CANx->MB[RxMessage->MBIndex].ID>>18;
	}
	//��ȡ��ַ
	for(i=0;i<len;i++)
  {  
	 if(i < 4)
	 (RxMessage->Data[0+i])=(word[0]>>((3-i)*8));
	 else									 //���ݴ洢ת��
	 (RxMessage->Data[0+i])=(word[1]>>((7-i)*8));
  }
	return TRUE;
}
/***********************************************************************************************
 ���ܣ�CAN �ж�����
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 CAN_IT   : �ж�Դ
       @arg CAN_IT_MB0 : ����0�ж�
       @arg CAN_IT_MB1 : ����1�ж�
       @arg CAN_IT_MB2 : ����2�ж�
       @arg ... 
			 NewState : �������߹ر� 
			 @arg ENABLE:  ����
       @arg DISABLE: �ر�
 ���أ�0
 ��⣺0
************************************************************************************************/
void CAN_ITConfig(CAN_Type* CANx, uint16_t CAN_IT, FunctionalState NewState)
{
  //�������
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IT_MB(CAN_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(CAN_IT <= CAN_IT_MB15)
	{
		(ENABLE == NewState)?(CANx->IMASK1 |= (1<<CAN_IT)):(CANx->IMASK1 &= ~(1<<CAN_IT));
	}
}
/***********************************************************************************************
 ���ܣ�CAN ���IT״̬
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 CAN_IT   : �ж�Դ
       @arg CAN_IT_MB0 : ����0�ж�
       @arg CAN_IT_MB1 : ����1�ж�
       @arg CAN_IT_MB2 : ����2�ж�
       @arg ... 
 ���أ�SET OR RESET
 ��⣺0
************************************************************************************************/
ITStatus CAN_GetITStatus(CAN_Type* CANx, uint16_t CAN_IT)
{
	ITStatus retval = RESET;
	
	if(CAN_IT <= CAN_IT_MB15)
	{
		((CANx->IFLAG1 >> CAN_IT) & 0x01)?(retval = SET):(retval = RESET);
	}
	return retval;
}
/***********************************************************************************************
 ���ܣ�CAN ����ж�Pending
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
			 CAN_IT   : �ж�Դ
       @arg CAN_IT_MB0 : ����0�ж�
       @arg CAN_IT_MB1 : ����1�ж�
       @arg CAN_IT_MB2 : ����2�ж�
       @arg ... 
 ���أ�0
 ��⣺0
************************************************************************************************/
void CAN_ClearITPendingBit(CAN_Type* CANx, uint16_t CAN_IT)
{
  //�������
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IT_MB(CAN_IT));
	
	if(CAN_IT <= CAN_IT_MB15)
	{
		CANx->IFLAG1 |= (1<<CAN_IT);
	}
}
/***********************************************************************************************
 ���ܣ�CAN ��������ж�Pending
 �βΣ�CAN_Type: CAN�ṹ
       @arg CAN0 : CAN0ģ��
			 @arg CAN1 : CAN1ģ��
 ���أ�SET OR RESET
 ��⣺0
************************************************************************************************/
void CAN_ClearAllITPendingBit(CAN_Type* CANx)
{
  //�������
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	
	CANx->IFLAG1 = 0xFFFF;
}

/*
static const CAN_MapTypeDef CAN_Check_Maps[] = 
{ 
    {0, 2, 0,12,13}, //CAN0_TX_PA12_RX_PA13
    {0, 2, 1,18,19}, //CAN0_TX_PB18_RX_PB19
    {1, 2, 4,24,25}, //CAN1_TX_PE24_RX_PE25
    {1, 2, 2,17,16}, //CAN1_TX_PC17_RX_PC16
};
void CAN_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(CAN_Check_Maps)/sizeof(CAN_MapTypeDef);i++)
	{
		value = CAN_Check_Maps[i].CAN_Index <<0;
		value|= CAN_Check_Maps[i].CAN_Alt_Index <<2;
		value|= CAN_Check_Maps[i].CAN_GPIO_Index <<5;
		value|= CAN_Check_Maps[i].CAN_TX_Pin_Index<<8;
		value|= CAN_Check_Maps[i].CAN_RX_Pin_Index<<14;
		printf("(0x%08xU)\r\n",value);
	}
}
*/




