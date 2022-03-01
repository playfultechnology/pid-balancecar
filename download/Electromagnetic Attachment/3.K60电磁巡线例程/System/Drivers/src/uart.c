/**
  ******************************************************************************
  * @file    uart.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.25
  * @brief   ����K60�̼��� UART ���� ������
  ******************************************************************************
  */
#include "uart.h"
#include "sys.h"
#include "string.h"
//���ͽṹ
UART_TxSendTypeDef UART_TxIntStruct1;
UART_Type* UART_DebugPort = NULL;

void UART_DebugPortInit(uint32_t UARTxMAP,uint32_t UART_BaudRate)
{
	UART_InitTypeDef UART_DebugInitStruct1;
	UART_MapTypeDef *pUART_Map = (UART_MapTypeDef*)&(UARTxMAP);
	//����Ĭ�ϵĵ���UART����
	UART_DebugInitStruct1.UART_BaudRate = UART_BaudRate;
	UART_DebugInitStruct1.UARTxMAP = UARTxMAP;
	//�ҳ���Ӧ��UART�˿�
	switch(pUART_Map->UART_Index)
	{
			case 0:
					UART_DebugPort = UART0;
					break;
			case 1:
					UART_DebugPort = UART1;
					break;
			case 2:
					UART_DebugPort = UART2;
					break;
			case 3:
					UART_DebugPort = UART3;
					break;
			case 4:
					UART_DebugPort = UART4;
					break;
			default:
					UART_DebugPort = NULL;
					break;
	}
	UART_Init(&UART_DebugInitStruct1);
}
/***********************************************************************************************
 ���ܣ���ʼ������
 �βΣ�UART_InitStruct UART��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void UART_Init(UART_InitTypeDef* UART_InitStruct)
{
  UART_Type* UARTx = NULL;
	PORT_Type *UART_PORT = NULL;
  uint16_t sbr;
	uint8_t brfa; 
	uint32_t clock;
	UART_MapTypeDef *pUART_Map = NULL;
	pUART_Map = (UART_MapTypeDef*)&(UART_InitStruct->UARTxMAP);
  //������
	assert_param(IS_UART_MAP(UART_InitStruct->UARTxMAP));
	//�ҳ���Ӧ��UART�˿�
	switch(pUART_Map->UART_Index)
	{
			case 0:
					SIM->SCGC4|=SIM_SCGC4_UART0_MASK;
					UARTx = UART0;
					break;
			case 1:
					SIM->SCGC4|=SIM_SCGC4_UART1_MASK;
					UARTx = UART1;
					break;
			case 2:
					SIM->SCGC4|=SIM_SCGC4_UART2_MASK;
					UARTx = UART2;
					break;
			case 3:
					SIM->SCGC4|=SIM_SCGC4_UART3_MASK;
					UARTx = UART3;
					break;
			case 4:
					SIM->SCGC1|=SIM_SCGC1_UART4_MASK;
					UARTx = UART4;
					break;
			default:
					UARTx = NULL;
					break;
	}
	 //�ҳ� PORT�˿� ��ʹ��ʱ��
	switch(pUART_Map->UART_GPIO_Index )
	{
		case 0:
			SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
			UART_PORT = PORTA;
			break;
		case 1:
			SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
			UART_PORT = PORTB;
			break;
		case 2:
			SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
			UART_PORT = PORTC;
			break;
		case 3:
			SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
			UART_PORT = PORTD;
			break;
		case 4:
			SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
			UART_PORT = PORTE;
			break;
		default:
			break;
	}
	//���ö�Ӧ����Ϊ����ģʽ
	UART_PORT->PCR[pUART_Map->UART_RX_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	UART_PORT->PCR[pUART_Map->UART_RX_Pin_Index] |= PORT_PCR_MUX(pUART_Map->UART_Alt_Index);
	UART_PORT->PCR[pUART_Map->UART_TX_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	UART_PORT->PCR[pUART_Map->UART_TX_Pin_Index] |= PORT_PCR_MUX(pUART_Map->UART_Alt_Index);
	//���ô���Ƶ��
	GetCPUInfo();  //����ϵͳʱ��
  clock = CPUInfo.BusClock;
	if((uint32_t)UARTx == UART0_BASE||(uint32_t)UARTx == UART1_BASE) 
	{
		clock = CPUInfo.CoreClock; //UART0 UART1ʹ��CoreClock
	}
	sbr = (uint16_t)((clock)/((UART_InitStruct->UART_BaudRate)*16));
	brfa = ((clock*2)/(UART_InitStruct->UART_BaudRate)-(sbr*32));
	UARTx->BDH |= ((sbr>>8)&UART_BDH_SBR_MASK);//���ø�5λ������
	UARTx->BDL = (sbr&UART_BDL_SBR_MASK);//���õ�8λ����
	UARTx->C4 |= brfa&(UART_BDL_SBR_MASK>>3);//����С��λ
	//����uart���ƼĴ�����ʵ�ֻ����İ�λ���书��
  UARTx->C2 &= ~(UART_C2_RE_MASK|UART_C2_TE_MASK);	 //��ֹ���ͽ���
	UARTx->C1 &= ~UART_C1_M_MASK;                      //��������λ��Ϊ8λ
	UARTx->C1 &= ~(UART_C1_PE_MASK);                   //����Ϊ����żУ��λ
	UARTx->S2 &= ~UART_S2_MSBF_MASK;                   //����Ϊ���λ���ȴ���
	//ʹ�ܽ������뷢����
	UARTx->C2|=(UART_C2_RE_MASK|UART_C2_TE_MASK);	 //�������ݷ��ͽ���,�μ��ֲ�1221ҳ
	//��¼��󻺳�����
	UART_TxIntStruct1.MaxBufferSize = MAX_TX_BUF_SIZE;
}

/***********************************************************************************************
 ���ܣ������жϿ���
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 UART_IT : ֧�ֵ��ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
void UART_ITConfig(UART_Type* UARTx, uint16_t UART_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_UART_IT(UART_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	switch(UART_IT)
	{
		case UART_IT_TDRE:
			(ENABLE == NewState)?(UARTx->C2 |= UART_C2_TIE_MASK):(UARTx->C2 &= ~UART_C2_TIE_MASK);
			break;
		case UART_IT_TC:
			(ENABLE == NewState)?(UARTx->C2 |= UART_C2_TCIE_MASK):(UARTx->C2 &= ~UART_C2_TCIE_MASK);
			break;
		case UART_IT_RDRF:
			(ENABLE == NewState)?(UARTx->C2 |= UART_C2_RIE_MASK):(UARTx->C2 &= ~UART_C2_RIE_MASK);
			break;
		case UART_IT_IDLE:
			(ENABLE == NewState)?(UARTx->C2 |= UART_C2_ILIE_MASK):(UARTx->C2 &= ~UART_C2_ILIE_MASK);
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�����жϱ�־
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 UART_IT : ֧�ֵ��ж�
 ���أ�0
 ��⣺0
************************************************************************************************/
ITStatus UART_GetITStatus(UART_Type* UARTx, uint16_t UART_IT)
{
	ITStatus retval;
	//�������
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_UART_IT(UART_IT));
	
	switch(UART_IT)
	{
		case UART_IT_TDRE:
			(UARTx->S1 & UART_S1_TDRE_MASK)?(retval = SET):(retval = RESET);
			break;
		case UART_IT_TC:
			(UARTx->S1 & UART_S1_TC_MASK)?(retval = SET):(retval = RESET);
			break;
		case UART_IT_RDRF:
			(UARTx->S1 & UART_S1_RDRF_MASK)?(retval = SET):(retval = RESET);	
			break;
		case UART_IT_IDLE:
			(UARTx->S1 & UART_S1_IDLE_MASK)?(retval = SET):(retval = RESET);			
			break;
		default:break;
	}
	return retval;
}


/***********************************************************************************************
 ���ܣ����ڷ���һ���ֽ�
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 Data : 0-0xFF ���͵�����
 ���أ�0
 ��⣺0
************************************************************************************************/
void UART_SendData(UART_Type* UARTx,uint8_t Data)
{
	while(!(UARTx->S1 & UART_S1_TDRE_MASK));
	UARTx->D = (uint8_t)Data;
}
/***********************************************************************************************
 ���ܣ�ʹ���жϷ��ʹ�������
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 *DataBuf : ���͵����� ������ָ��
			  Len     : ���͵����ݳ���
 ���أ�0
 ��⣺0
************************************************************************************************/
void UART_SendDataInt(UART_Type* UARTx,uint8_t* pBuffer,uint8_t NumberOfBytes)
{
	//�������
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	
	//�ڴ濽��
	memcpy(UART_TxIntStruct1.TxBuf,pBuffer,NumberOfBytes);
	UART_TxIntStruct1.Length = NumberOfBytes;
	UART_TxIntStruct1.Offset = 0;
	UART_TxIntStruct1.IsComplete = FALSE;
	//ʹ���жϷ�ʽ���� ��ʹ��DMA
	UARTx->C5 &= ~UART_C5_TDMAS_MASK; 
	//ʹ�ܴ����ж�
	UARTx->C2 |= UART_C2_TIE_MASK;
}
/***********************************************************************************************
 ���ܣ�����UART DMA֧��s
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 UART_DMAReq : DMA�ж�Դ

			 NewState    : ʹ�ܻ��߹ر�
			 @arg  ENABLE : ʹ��
			 @arg  DISABLE: ��ֹ
 ���أ�0
 ��⣺��ҪDMA������֧�� ��Ҫʹ��DMA�����е� Iscomplete�����ж��Ƿ������
************************************************************************************************/
void UART_DMACmd(UART_Type* UARTx, uint16_t UART_DMAReq, FunctionalState NewState)
{
	//�������
	assert_param(IS_UART_IT(UART_DMAReq));
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(UART_DMAReq)
	{
		case UART_DMAReq_Tx:
			(NewState == ENABLE)?(UARTx->C5 |= UART_C5_TDMAS_MASK):(UARTx->C5 &= ~UART_C5_TDMAS_MASK);
			break;
		case UART_DMAReq_Rx:
			(NewState == ENABLE)?(UARTx->C5 |= UART_C5_RDMAS_MASK):(UARTx->C5 &= ~UART_C5_RDMAS_MASK);
			break;
			default:break;
	}
}
/***********************************************************************************************
 ���ܣ�ʹ���жϷ�ʽ ���ʹ������� �жϹ���
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4
 ���أ�0
 ��⣺�����������жϷ���ʱ �ڶ�Ӧ�Ĵ����ж��е��ô˹���
************************************************************************************************/
void UART_SendDataIntProcess(UART_Type* UARTx)
{
	if((UARTx->S1 & UART_S1_TDRE_MASK) && (UARTx->C2 & UART_C2_TIE_MASK))
	{
		if(UART_TxIntStruct1.IsComplete == FALSE)
		{
			UARTx->D = UART_TxIntStruct1.TxBuf[UART_TxIntStruct1.Offset++];
			if(UART_TxIntStruct1.Offset >= UART_TxIntStruct1.Length)
			{
					UART_TxIntStruct1.IsComplete = TRUE;
				  //�رշ����ж�
					UARTx->C2 &= ~UART_C2_TIE_MASK;
			}
		} 
	}
}
/***********************************************************************************************
 ���ܣ����ڽ���һ���ֽ�
 �βΣ�UART_Type ����ѡ��
			 @arg  UART0: ����0
			 @arg  UART1: ����1
			 @arg  UART2: ����2
			 @arg  UART3: ����3
			 @arg  UART4: ����4

			 *ch : ���յ����ֽ� ����ָ��
 ���أ�0 ����ʧ��
       1 ���ճɹ�
 ��⣺0
************************************************************************************************/
uint8_t UART_ReceiveData(UART_Type *UARTx,uint8_t *ch)
{
	if((UARTx->S1 & UART_S1_RDRF_MASK)!= 0)//�жϽ��ջ������Ƿ���
	{
		*ch = (UARTx->D);	//��������
		 return 1; 		  	//���ܳɹ�
	}
	return 0;			      //�����ʱ������ʧ��
}
//�ڲ�����Ϊʵ��UART_printf
static void UART_puts(char *pch)
{
	while(*pch != '\0')
	{
		UART_SendData(UART_DebugPort,*pch);
		pch++;
	}
}
//�ڲ�����Ϊʵ��UART_printf
static void printn(unsigned int n, unsigned int b)
{
	static char *ntab = "0123456789ABCDEF";
	unsigned int a, m;
	if (n / b)
	{
		a = n / b;
		printn(a, b);  
	}
	m = n % b;
	UART_SendData(UART_DebugPort,ntab[m]);
}
/***********************************************************************************************
 ���ܣ�UART ��ʽ�����
 �βΣ�fmt �����ַ���ָ��          
 ���أ�0
 ��⣺������C��׼���е�printf ����ֻ֧�� %d %l %o %x %s
************************************************************************************************/
void UART_printf(char *fmt, ...)
{
    char c;
    unsigned int *adx = (unsigned int*)(void*)&fmt + 1;
_loop:
    while((c = *fmt++) != '%')
		{
        if (c == '\0') return;
        UART_SendData(UART_DebugPort,c);
    }
    c = *fmt++;
    if (c == 'd' || c == 'l')
		{
        printn(*adx, 10);
    }
    if (c == 'o' || c == 'x')
		{
        printn(*adx, c=='o'? 8:16 );
    }
    if (c == 's')
		{
			UART_puts((char*)*adx);
    }
    adx++;
    goto _loop;
}
/***********************************************************************************************
 ���ܣ���ӡ��������Ϣ
 �βΣ�0          
 ���أ�0
 ��⣺�����Freescale Kinetisϵ��
************************************************************************************************/
void DisplayCPUInfo(void)
{
    //��ӡ�̼���汾
    UART_printf("CH Kinetis FW Lib\r\n");
    UART_printf("Version:%d.%d.%d build %s\r\n",
               CHK_VERSION, CHK_SUBVERSION, CHK_REVISION, __DATE__);
    UART_printf("2010 - 2013 Copyright by yandld\r\n");
	//��ӡ��λ��Ϣ
	switch(CPUInfo.ResetState)
	{
		case 1: UART_printf("Software Reset\r\n");           break;
		case 2: UART_printf("Core Lockup Event Reset\r\n");  break;
		case 3: UART_printf("JTAG Reset\r\n");               break;
		case 4: UART_printf("Power-on Reset\r\n");           break;
		case 5: UART_printf("External Pin Reset\r\n");       break;
		case 6: UART_printf("Watchdog(COP) Reset\r\n");      break;
		case 7: UART_printf("Loss of Clock Reset\r\n");      break;
		case 8: UART_printf("Low-voltage Detect Reset\r\n"); break;
		case 9: UART_printf("LLWU Reset\r\n");               break;
	}
	//��ӡKinetisϵ���ͺ�
	switch(CPUInfo.FamilyType)
	{
		case 10: UART_printf("Family:K10\r\n"); break;
		case 20: UART_printf("Family:K20\r\n"); break;
		case 30: UART_printf("Family:K30\r\n"); break;
		case 40: UART_printf("Family:K40\r\n"); break;
		case 50: UART_printf("Family:K50\r\n"); break;
		case 53: UART_printf("Family:K53\r\n"); break;
		case 60: UART_printf("Family:K60\r\n"); break;
		default: UART_printf("\nUnrecognized Kinetis family device.\n"); break;  
	}
	//��ӡ��װ��Ϣ
	UART_printf("PinCnt:%d\r\n",CPUInfo.PinCnt);
	//��ӡSiliconRevID
	UART_printf("SiliconRevID:%d.%d\r\n",CPUInfo.SiliconRev/10,CPUInfo.SiliconRev%10);
	//��ӡPFlash��С
	UART_printf("PFlash Size: %dKB\r\n",CPUInfo.PFlashSize/1024);
	//��ӡFlexNVM��С
	UART_printf("FlexNVM Size: %dKB\r\n",CPUInfo.FlexNVMSize/1024);
	//��ӡRAM ��С
	UART_printf("RAM Size :%dKB\r\n",CPUInfo.RAMSize/1024);
	//��ӡCoreClock
	UART_printf("CoreClock: %dHz\r\n",CPUInfo.CoreClock);
	//��ӡBusClock
	UART_printf("BusClock: %dHz\r\n",CPUInfo.BusClock);
	//��ӡFlexBusClock
	UART_printf("FlexBusClock: %dHz\r\n",CPUInfo.FlexBusClock);
	//��ӡFlashClock
	UART_printf("FlashClock: %dHz\r\n",CPUInfo.FlashClock);
}

/*
static const UART_MapTypeDef2 UART_Check_Maps[] = 
{ 
    {0, 2, 0, 1, 2, 0, 0}, //UART0_RX_PA1_TX_PA2
    {0, 3, 0,14,15, 0, 0}, //UART0_RX_PA14_TX_PA15
    {0, 3, 1,16,17, 0, 0}, //UART0_RX_PB16_TX_PB17
    {0, 3, 3, 6, 7, 0, 0}, //UART0_RX_PD6_TX_PD7
    {1, 3, 4, 0, 1, 0, 0}, //UART1_RX_PE0_TX_PE1
    {1, 3, 2, 3, 4, 0, 0}, //UART1_RX_C3_TX_C4
    {2, 3, 3, 2, 3, 0, 0}, //UART2_RX_D2_TX_D3
    {3, 3, 1,10,11, 0, 0}, //UART3_RX_B10_TX_B11
    {3, 3, 2,16,17, 0, 0}, //UART3_RX_C16_TX_C17
    {3, 3, 4, 4, 5, 0, 0}, //UART3_RX_E4_TX_E5
    {4, 3, 4,24,25, 0, 0}, //UART4_RX_E24_TX_E25
    {4, 3, 2,14,15, 0, 0}, //UART4_RX_C14_TX_C15
};

void UART_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(UART_Check_Maps)/sizeof(UART_MapTypeDef2);i++)
	{
		value = UART_Check_Maps[i].UART_Index<<0;
		value |=  UART_Check_Maps[i].UART_Alt_Index <<3;
		value |=  UART_Check_Maps[i].UART_GPIO_Index<<6;
		value |=  UART_Check_Maps[i].UART_RX_Pin_Index<<9;
		value |=  UART_Check_Maps[i].UART_TX_Pin_Index<<14;
		value |=  UART_Check_Maps[i].UART_CTS_Pin_Index<<19;	
		value |=  UART_Check_Maps[i].UART_RTS_Pin_Index<<24;	
		printf("(0x%08XU)\r\n",value);
	}
}
*/
