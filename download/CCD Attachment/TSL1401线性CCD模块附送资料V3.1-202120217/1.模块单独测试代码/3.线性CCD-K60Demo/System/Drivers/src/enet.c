/**
  ******************************************************************************
  * @file    enet.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.6.23
  * @brief   ����K60�̼��� ��̫�� �����ļ�
  ******************************************************************************
  */
#include "enet.h"

#ifdef DEBUG_PRINT
#include "uart.h"
#endif

//������̫��DMA������
static  uint8_t xENETTxDescriptors_unaligned[ ( 1 * sizeof( NBUF ) ) + 16 ];
static  uint8_t pxENETRxDescriptors_unaligned[ ( CFG_NUM_ENET_RX_BUFFERS * sizeof( NBUF ) ) + 16 ];
static NBUF *pxENETTxDescriptor;
static NBUF *pxENETRxDescriptors;

//��̫�����ջ�����
static uint8_t ucENETRxBuffers[ ( CFG_NUM_ENET_RX_BUFFERS * CFG_ENET_BUFFER_SIZE ) + 16 ];
static uint32_t uxNextRxBuffer = 0;

/***********************************************************************************************
 ���ܣ���̫����������ʼ��
 �βΣ�0
 ���أ�0
 ��⣺��̫��ģ��ͨ�� ������ ������(����USB) ��������̫��
************************************************************************************************/
static void ENET_BDInit(void)
{
  unsigned long ux;
	unsigned char *pcBufPointer;
	//Ѱ��16�ֽڶ���ռ�
	pcBufPointer = &( xENETTxDescriptors_unaligned[ 0 ] );
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETTxDescriptor = ( NBUF * ) pcBufPointer;	
	//Ѱ��16�ֽڶ���ռ�
	pcBufPointer = &( pxENETRxDescriptors_unaligned[ 0 ] );
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETRxDescriptors = ( NBUF * ) pcBufPointer;

	pxENETTxDescriptor->length = 0;
	pxENETTxDescriptor->status = 0;
	pxENETTxDescriptor->ebd_status = TX_BD_IINS | TX_BD_PINS;
	//Ѱ��16�ֽڶ���ռ�
	pcBufPointer = &( ucENETRxBuffers[ 0 ] );
	while((( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	//��ʼ������������
	for( ux = 0; ux < CFG_NUM_ENET_RX_BUFFERS; ux++ )
	{
	    pxENETRxDescriptors[ ux ].status = RX_BD_E;
	    pxENETRxDescriptors[ ux ].length = 0;
      pxENETRxDescriptors[ ux ].data = (uint8_t *)__REV((uint32_t)pcBufPointer);
	    pcBufPointer += CFG_ENET_BUFFER_SIZE;
	    pxENETRxDescriptors[ ux ].bdu = 0x00000000;
	    pxENETRxDescriptors[ ux ].ebd_status = RX_BD_INT;
	}
	//���һ������������ΪWarp
	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
	//��0��������ʼ
	uxNextRxBuffer = 0;
	
}
/***********************************************************************************************
 ���ܣ�����MAC��ַ
 �βΣ�0
 ���أ�0
 ��⣺0
************************************************************************************************/
uint8_t ENET_HashAddress(const uint8_t* addr)
{
  uint32_t crc;
  uint8_t byte;
  int i, j;
  crc = 0xFFFFFFFF;
  for(i=0; i<6; ++i)
  {
    byte = addr[i];
    for(j=0; j<8; ++j)
    {
      if((byte & 0x01)^(crc & 0x01))
      {
        crc >>= 1;
        crc = crc ^ 0xEDB88320;
      }
      else
        crc >>= 1;
      byte >>= 1;
    }
  }
  return (uint8_t)(crc >> 26);
}
/***********************************************************************************************
 ���ܣ�����MAC��ַ
 �βΣ�0
 ���أ�0
 ��⣺0
************************************************************************************************/
void ENET_SetAddress(const uint8_t *pa)
{
  uint8_t crc;
  //���������ַ
  ENET->PALR = (uint32_t)((pa[0]<<24) | (pa[1]<<16) | (pa[2]<<8) | pa[3]);
  ENET->PAUR = (uint32_t)((pa[4]<<24) | (pa[5]<<16));
  
  //���������ַ���㲢���ö�����ַ��ϣ�Ĵ�����ֵ
  crc = ENET_HashAddress(pa);
  if(crc >= 32)
    ENET->IAUR |= (uint32_t)(1 << (crc - 32));
  else
    ENET->IALR |= (uint32_t)(1 << crc);
}
/***********************************************************************************************
 ���ܣ���ʼ����̫��ģ��
 �βΣ�0
 ���أ�0�ɹ� 1ʧ��
 ��⣺0
************************************************************************************************/
uint8_t ENET_Init(ENET_InitTypeDef* ENET_InitStrut)
{
	uint16_t usData;
	uint16_t timeout = 0;
	//��ʼ���ṹ��
	//enetdev.recflag = 0;
	//enetdev.linkstate = LINK_STATE_OFF; 
  //ʹ��ENETʱ��
  SIM->SCGC2 |= SIM_SCGC2_ENET_MASK;
  //����������MPU������
  MPU->CESR = 0;         
  //��������������ʼ��
  ENET_BDInit();
	//��PORTʱ��
	SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
	//����Ҫ����
	MCG->C2 &= ~MCG_C2_EREFS_MASK;
	//��λ��̫��
	ENET->ECR = ENET_ECR_RESET_MASK;
	for( usData = 0; usData < 100; usData++ )
	{
		//__NOP;
	}
  //��ʼ��MII�ӿ�
  ENET_MiiInit();  

	//�����ж�
	NVIC_EnableIRQ(ENET_Transmit_IRQn);
	NVIC_EnableIRQ(ENET_Receive_IRQn);
	NVIC_EnableIRQ(ENET_Error_IRQn);
  //ʹ��GPIO���Ÿ��ù���
  PORTB->PCR[0]  = PORT_PCR_MUX(4); 
  PORTB->PCR[1]  = PORT_PCR_MUX(4); 
	PORTA->PCR[12] =  PORT_PCR_MUX(4);  
	PORTA->PCR[13] =  PORT_PCR_MUX(4);  
	PORTA->PCR[14] =  PORT_PCR_MUX(4);  
	PORTA->PCR[15] =  PORT_PCR_MUX(4);  
	PORTA->PCR[16] =  PORT_PCR_MUX(4);  
	PORTA->PCR[17] =  PORT_PCR_MUX(4);  
  //�ȴ�PHY�շ�����λ���
  do
  {
    DelayMs(10);
		timeout++;
		if(timeout > 500) return 1;
    usData = 0xffff;
    ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
        
  } while( usData == 0xffff );

#ifdef DEBUG_PRINT
  UART_printf("PHY_PHYIDR1=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYIDR2, &usData );
  UART_printf("PHY_PHYIDR2=0x%X\r\n",usData); 
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_ANLPAR, &usData );
  UART_printf("PHY_ANLPAR=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_ANLPARNP, &usData );
  UART_printf("PHY_ANLPARNP=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYSTS, &usData );
  UART_printf("PHY_PHYSTS=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_MICR, &usData );
  UART_printf("PHY_MICR=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_MISR, &usData );
  UART_printf("PHY_MISR=0x%X\r\n",usData);
#endif 
  //��ʼ�Զ�Э��
  ENET_MiiWrite(CFG_PHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );

#ifdef DEBUG_PRINT
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMCR, &usData );
  UART_printf("PHY_BMCR=0x%X\r\n",usData);
#endif 
  //�ȴ��Զ�Э�����
  do
  {
    DelayMs(10);
		timeout++;
		if(timeout > 500) return 1;
    ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMSR, &usData );

  } while( !( usData & PHY_BMSR_AN_COMPLETE ) );
  //����Э�̽������ENETģ��
	usData = 0;
	ENET_MiiRead(CFG_PHY_ADDRESS, PHY_STATUS, &usData );	
  
  //������������ַ��ϣ�Ĵ���
  ENET->IALR = 0;
  ENET->IAUR = 0;
  ENET->GALR = 0;
  ENET->GAUR = 0;
  //����ENETģ��MAC��ַ
  ENET_SetAddress(ENET_InitStrut->pMacAddress);
  //���ý��տ��ƼĴ�������󳤶ȡ�RMIIģʽ������CRCУ���
  ENET->RCR = ENET_RCR_MAX_FL(CFG_ENET_MAX_PACKET_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK | ENET_RCR_RMII_MODE_MASK;

  //������ͽ��տ���
  ENET->TCR = 0;
  //ͨѶ��ʽ����
  if( usData & PHY_DUPLEX_STATUS )
  {
    //ȫ˫��
    ENET->RCR &= (unsigned long)~ENET_RCR_DRT_MASK;
    ENET->TCR |= ENET_TCR_FDEN_MASK;
		#ifdef DEBUG_PRINT
			UART_printf("ȫ˫��\r\n");
		#endif 
  }
  else
  {
    //��˫��
    ENET->RCR |= ENET_RCR_DRT_MASK;
    ENET->TCR &= (unsigned long)~ENET_TCR_FDEN_MASK;
		#ifdef DEBUG_PRINT
		UART_printf("��˫��\r\n");
		#endif 
  }
  //ͨ����������
  if( usData & PHY_SPEED_STATUS )
  {
    //10Mbps
    ENET->RCR |= ENET_RCR_RMII_10T_MASK;
  }

  //ʹ����ǿ�ͻ�����������
  ENET->ECR = ENET_ECR_EN1588_MASK;

	//���ý��ջ�������С
	ENET->MRBR = ENET_MRBR_R_BUF_SIZE(CFG_ENET_MAX_PACKET_SIZE);

	//ָ���λ��������������׵�ַ(RX)
	ENET->RDSR = (uint32_t)  pxENETRxDescriptors;

	//ָ���λ��������������׵�ַ(TX)
	ENET->TDSR = (uint32_t) pxENETTxDescriptor;

	//��������жϱ�־
	ENET->EIR = ( uint32_t ) 0xFFFFFFFF;

	//ʹ���ж�
	ENET->EIMR = ENET_EIR_TXF_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_RXB_MASK | ENET_EIMR_UN_MASK | ENET_EIMR_RL_MASK | ENET_EIMR_LC_MASK | ENET_EIMR_BABT_MASK | ENET_EIMR_BABR_MASK | ENET_EIMR_EBERR_MASK;

	//ʹ��MACģ��
	ENET->ECR |= ENET_ECR_ETHEREN_MASK;
  //�������ջ�����Ϊ��
	ENET->RDAR = ENET_RDAR_RDAR_MASK;
	//�������״̬
//	enetdev.linkstate =  ENET_MiiLinkState();
	return 0;
}
/***********************************************************************************************
 ���ܣ����������
 �βΣ�0
 ���أ�0
 ��⣺0
************************************************************************************************/
void ENET_MiiInit(void)
{
	uint8_t i;
	GetCPUInfo();
	i = (CPUInfo.BusClock/1000)/1000;
  ENET->MSCR = 0 | ENET_MSCR_MII_SPEED((2*i/5)+1);
}

//�����д������
uint8_t ENET_MiiWrite(uint16_t phy_addr, uint16_t reg_addr, uint16_t data)
{
	uint32_t timeout;
  //���MII�ж��¼�
	ENET->EIR = ENET_EIR_MII_MASK;
  //��ʼ��MII����֡�Ĵ���
	ENET->MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x01)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02)
            | ENET_MMFR_DATA(data);
  //�ȴ�MII��������ж��¼�
  for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
  {
    if (ENET->EIR & ENET_EIR_MII_MASK)
      break;
  }
  if(timeout == MII_TIMEOUT) 
    return 1;
  //���MII�ж��¼�
  ENET->EIR = ENET_EIR_MII_MASK;
  return 0;
}
/***********************************************************************************************
 ���ܣ�RMII�� ��ȡ����
 �βΣ�phy_addr: �ӿڵ�ַ  reg_addr:Ҫ��ȡ�ļĴ���  *data:����
 ���أ�0 �ɹ�  1ʧ��
 ��⣺0
************************************************************************************************/
uint8_t ENET_MiiRead(uint16_t phy_addr, uint16_t reg_addr, uint16_t *data)
{
	uint32_t timeout;
	//���MII�ж��¼�
	ENET->EIR = ENET_EIR_MII_MASK;
	//��ʼ��MII����֡�Ĵ���
  ENET->MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x2)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02);
  
	//�ȴ�MII��������ж��¼�
	for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
  {
    if (ENET->EIR & ENET_EIR_MII_MASK)
      break;
  }
  if(timeout == MII_TIMEOUT) 
    return 1;
  //���MII�ж��¼�
  ENET->EIR = ENET_EIR_MII_MASK;
  *data = ENET->MMFR & 0x0000FFFF;
  return 0;
}
/***********************************************************************************************
 ���ܣ��鿴��������״̬
 �βΣ�0
 ���أ�ENET_PHY_LINK_STATE
					LINK_STATE_ON,
					LINK_STATE_OFF,
 ��⣺0
************************************************************************************************/
uint8_t ENET_MiiLinkState(void)
{
	uint16_t reg = 0;
	ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMSR, &reg );
	if(reg & 0x0004)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/***********************************************************************************************
 ���ܣ�����һ����̫֡ 
 �βΣ�*ch:����ָ��   len:����
 ���أ�0
 ��⣺0
************************************************************************************************/
void ENET_MacSendData(uint8_t *ch, uint16_t len)
{
  //��鵱ǰ���ͻ������������Ƿ����
	while( pxENETTxDescriptor->status & TX_BD_R )
	{
		
	}
  //���÷��ͻ�����������
  pxENETTxDescriptor->data = (uint8_t *)__REV((uint32_t)ch);		
  pxENETTxDescriptor->length = __REVSH(len);
	pxENETTxDescriptor->bdu = 0x00000000;
	pxENETTxDescriptor->ebd_status = TX_BD_INT | TX_BD_TS;// | TX_BD_IINS | TX_BD_PINS;
	pxENETTxDescriptor->status = ( TX_BD_R | TX_BD_L | TX_BD_TC | TX_BD_W );
  //ʹ�ܷ���
  ENET->TDAR = ENET_TDAR_TDAR_MASK;
}

/***********************************************************************************************
 ���ܣ�����һ����̫֡
 �βΣ�*ch:����ָ�� 
 ���أ�֡����
 ��⣺0
************************************************************************************************/
uint16_t ENET_MacRecData(uint8_t *ch)
{
	uint16_t len = 0;
	ch = ch;
	//Ѱ�ҷǿյĻ�����������
	if((pxENETRxDescriptors[uxNextRxBuffer].status & RX_BD_E ) == 0)
	{
		//��ȡ����
		len =  __REVSH(pxENETRxDescriptors[ uxNextRxBuffer ].length);
		memcpy(ch,(uint8_t *)__REV((uint32_t)pxENETRxDescriptors[ uxNextRxBuffer ].data),len);
		//��ʾ�Ѿ���ȡ�Ļ���������
		pxENETRxDescriptors[uxNextRxBuffer].status |= RX_BD_E;
		uxNextRxBuffer++;
		if( uxNextRxBuffer >= CFG_NUM_ENET_RX_BUFFERS )
		{
			uxNextRxBuffer = 0;
		}
		ENET->RDAR = ENET_RDAR_RDAR_MASK;
	}
	return len;
	
}
/***********************************************************************************************
 ���ܣ���̫����������ж�
 �βΣ�
 ���أ�0
 ��⣺0
************************************************************************************************/
void ENET_Transmit_IRQHandler(void)
{
	ENET->EIR |= ENET_EIMR_TXF_MASK; 
}
/***********************************************************************************************
 ���ܣ���̫�������ж�
 �βΣ�
 ���أ�0
 ��⣺0
************************************************************************************************/
uint8_t gEnetFlag = 0;
void ENET_Receive_IRQHandler(void)
{
	ENET->EIR |= ENET_EIMR_RXF_MASK; 
	gEnetFlag = 1;
}

void ENET_Error_IRQHandler(void)
{
	//UART_printf("��̫������\r\n");
}


