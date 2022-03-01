
#include "include.h"
#include "MK60_UART.h"

/* ����6��ָ�����鱣�� UARTn_e �ĵ�ַ */
UART_MemMapPtr UARTN[5] = {UART0_BASE_PTR, UART1_BASE_PTR, UART2_BASE_PTR, UART3_BASE_PTR, UART4_BASE_PTR};


//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
//IAR����Ҫ��options -> C/C++compiler -> Preprocessor ��Ӻ궨�� _DLIB_FILE_DESCRIPTOR
#if 1
//#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
	int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
//�ض���fputc����  ʹ�ô���4��Ϊprintf�Ĵ�ӡ��
int fputc(int ch, FILE *f)
{
    while(!(UART_S1_REG(UARTN[3]) & UART_S1_TDRE_MASK));
    //��������
    UART_D_REG(UARTN[3]) = (uint8)ch;
	return ch;
}
#endif


//-------------------------------------------------------------------------*
//������: UART_Init
//��  ��: ��ʼ��UART
//��  ��: uratn:ģ�����磺UART0
//        baud: ������
//��  ��: ��
//��  ��: uart_init(UART4,115200);UART4��Ӧ���Ųο�UART.H�ļ�
//-------------------------------------------------------------------------*
void UART_Init (UART_Type * uratn, uint32 baud)
{
    register uint16 sbr, brfa;
    uint8 temp;
    uint32 sysclk;     //ʱ��

	uint8_t uart_num = UART_GetNum(uratn);
    /* ���� UART���ܵ� ���ùܽ� */
    switch(uart_num)
    {
    case 0:
        SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;      //ʹ�� UART0 ʱ��

        if(UART0_RX == PTA1)
        {
            PORTA_PCR1= PORT_PCR_MUX(2);       //ʹ��PTA1���ŵڶ����ܣ���UART0_RXD
        }
        else if(UART0_RX == PTA15)
        {
             PORTA_PCR15= PORT_PCR_MUX(3);       //ʹ��PTA15���ŵ�3����
        }
        else if(UART0_RX == PTB16)
        {
             PORTB_PCR16= PORT_PCR_MUX(3);       //ʹ��PTB16���ŵ�3����
        }
        else if(UART0_RX == PTD6)
        {
             PORTD_PCR6= PORT_PCR_MUX(3);       //ʹ��PTD6���ŵ�3����
        }
        else
        {
            break;
        }

        if(UART0_TX == PTA2)
        {
             PORTA_PCR2= PORT_PCR_MUX(2);       //ʹ��PTA2���ŵڶ�����
        }
        else if(UART0_TX == PTA14)
        {
             PORTA_PCR14= PORT_PCR_MUX(3);       //PTA14
        }
        else if(UART0_TX == PTB17)
        {
             PORTB_PCR17= PORT_PCR_MUX(3);       //PTB17
        }
        else if(UART0_TX == PTD7)
        {
             PORTD_PCR7= PORT_PCR_MUX(3);       //PTD7
        }
        else
        {
             break;
        }

        break;

    case 1:
        SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;

        if(UART1_RX == PTC3)
        {
             PORTC_PCR3= PORT_PCR_MUX(3);       //PTC3
        }
        else if(UART1_RX == PTE1)
        {
             PORTE_PCR1= PORT_PCR_MUX(3);       //PTE1
        }
        else
        {
            break;
        }

        if(UART1_TX == PTC4)
        {
             PORTC_PCR4= PORT_PCR_MUX(3);       //PTC4
        }
        else if(UART1_TX == PTE0)
        {
             PORTE_PCR0= PORT_PCR_MUX(3);       //PTE0
        }
        else
        {
            break;
        }

        break;

    case 2:
        SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
         PORTD_PCR3= PORT_PCR_MUX(3);       //PTD3
         PORTD_PCR2= PORT_PCR_MUX(3);       //PTD2
        break;

    case 3:
        SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;

        if(UART3_RX == PTB10)
        {
             PORTB_PCR10= PORT_PCR_MUX(3);       //PTB10
        }
        else if(UART3_RX == PTC16)
        {
             PORTC_PCR16= PORT_PCR_MUX(3);       //PTC16
        }
        else if(UART3_RX == PTE5)
        {
             PORTE_PCR5= PORT_PCR_MUX(3);       //PTE5
        }
        else
        {
             break;
        }

        if(UART3_TX == PTB11)
        {
             PORTB_PCR11= PORT_PCR_MUX(3);       //PTB11
        }
        else if(UART3_TX == PTC17)
        {
             PORTC_PCR17= PORT_PCR_MUX(3);       //PTC17
        }
        else if(UART3_TX == PTE4)
        {
             PORTE_PCR4= PORT_PCR_MUX(3);       //PTE4
        }
        else
        {
             break;
        }
        break;

    case 4:
        SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;

        if(UART4_RX == PTC14)
        {
             PORTC_PCR14= PORT_PCR_MUX(3);       //PTC14
        }
        else if(UART4_RX == PTE25)
        {
             PORTE_PCR25= PORT_PCR_MUX(3);       //PTE25
        }
        else
        {
            break;
        }

        if(UART4_TX == PTC15)
        {
             PORTC_PCR15= PORT_PCR_MUX(3);       //PTC15
        }
        else if(UART4_TX == PTE24)
        {
             PORTE_PCR24= PORT_PCR_MUX(3);       //PTE24
        }
        else
        {
             break;
        }
        break;
    default:
        break;
    }

    //���õ�ʱ��Ӧ�ý�ֹ���ͽ���
    UART_C2_REG(uratn) &= ~(0
                                   | UART_C2_TE_MASK
                                   | UART_C2_RE_MASK
                                  );


    //���ó�8λ��У��ģʽ
    //���� UART ���ݸ�ʽ��У�鷽ʽ��ֹͣλλ����ͨ������ UART ģ����ƼĴ��� C1 ʵ�֣�
    UART_C1_REG(uratn) |= (0
                                  //| UART_C2_M_MASK                    //9 λ�� 8 λģʽѡ�� : 0 Ϊ 8λ ��1 Ϊ 9λ��ע���˱�ʾ0����8λ�� �������9λ��λ8��UARTx_C3�
                                  //| UART_C2_PE_MASK                   //��żУ��ʹ�ܣ�ע���˱�ʾ���ã�
                                  //| UART_C2_PT_MASK                   //У��λ���� : 0 Ϊ żУ�� ��1 Ϊ ��У��
                                 );

    //���㲨���ʣ�����0��1ʹ���ں�ʱ�ӣ���������ʹ��busʱ��
    if ((uratn == UART0) || (uratn == UART1))
    {
        sysclk = core_clk * 1000*1000;                                   //�ں�ʱ��
    }
    else
    {
        sysclk =  bus_clk * 1000*1000;                                    //busʱ��(���ں�ʱ�ӵ�һ��)
    }

    //UART ������ = UART ģ��ʱ�� / (16 �� (SBR[12:0] + BRFA))
    //������ BRFA ������£� SBR = UART ģ��ʱ�� / (16 * UART ������)
    sbr = (uint16)(sysclk / (baud * 16));
    if(sbr > 0x1FFF)sbr = 0x1FFF;                                       //SBR �� 13bit�����Ϊ 0x1FFF

    //��֪ SBR ���� BRFA =  = UART ģ��ʱ�� / UART ������ - 16 ��SBR[12:0]
    brfa = (sysclk / baud)  - (sbr * 16);


    //д SBR
    temp = (uint16)(UART_BDH_REG(uratn)) & (uint16)(~UART_BDH_SBR_MASK);           //���� ��� SBR �� UARTx_BDH��ֵ
    UART_BDH_REG(uratn) = temp |  UART_BDH_SBR(sbr >> 8);        //��д��SBR��λ
    UART_BDL_REG(uratn) = UART_BDL_SBR(sbr);                     //��д��SBR��λ

    //д BRFD
    temp =(uint16)( UART_C4_REG(uratn)) & (uint16)(~UART_C4_BRFA_MASK) ;           //���� ��� BRFA �� UARTx_C4 ��ֵ
    UART_C4_REG(uratn) = temp |  UART_C4_BRFA(brfa);             //д��BRFA



    //����FIFO(FIFO���������Ӳ�������ģ������������)
    UART_PFIFO_REG(uratn) |= (0
                                     | UART_PFIFO_TXFE_MASK               //ʹ��TX FIFO(ע�ͱ�ʾ��ֹ)
                                     //| UART_PFIFO_TXFIFOSIZE(0)         //��ֻ����TX FIFO ��С��0Ϊ1�ֽڣ�1~6Ϊ 2^(n+1)�ֽ�
                                     | UART_PFIFO_RXFE_MASK               //ʹ��RX FIFO(ע�ͱ�ʾ��ֹ)
                                     //| UART_PFIFO_RXFIFOSIZE(0)         //��ֻ����RX FIFO ��С��0Ϊ1�ֽڣ�1~6Ϊ 2^(n+1)�ֽ�
                                    );

    /* �����ͺͽ��� */
    UART_C2_REG(uratn) |= (0
                                  | UART_C2_TE_MASK                     //����ʹ��
                                  | UART_C2_RE_MASK                     //����ʹ��
                                  //| UART_C2_TIE_MASK                  //�����жϻ�DMA��������ʹ�ܣ�ע���˱�ʾ���ã�
                                  //| UART_C2_TCIE_MASK                 //��������ж�ʹ�ܣ�ע���˱�ʾ���ã�
                                  | UART_C2_RIE_MASK                    //�������жϻ�DMA��������ʹ�ܣ�ע���˱�ʾ���ã�
                                 );


}



//-------------------------------------------------------------------------*
//������: UART_PutChar
//��  ��: ����һ���ֽ�
//��  ��: uratn:ģ�����磺UART0
//         ch: ���͵��ֽ�
//��  ��: ��
//��  ��: UART_PutChar (UART4, 0x66);
//-------------------------------------------------------------------------*
void UART_PutChar(UART_Type * uratn, char ch)
{
    //�ȴ����ͻ�������
    while(!(UART_S1_REG(uratn) & UART_S1_TDRE_MASK));
    //��������
    UART_D_REG(uratn) = (uint8)ch;
}


//-------------------------------------------------------------------------*
//������: UART_PutBuff
//��  ��: ����ָ��len���ֽڳ������� ������ NULL Ҳ�ᷢ�ͣ�
//��  ��: uratn:ģ�����磺UART0
//        buff: ���͵ĵ�ַ��
//        len : ����ָ������
//��  ��: ��
//��  ��: UART_PutBuff (UART4, "123456789",5);ʵ�ʷ���5���ֽڡ�1����2����3����4����5��
//-------------------------------------------------------------------------*
void UART_PutBuff(UART_Type * uratn, uint8_t *buff, uint32 len)
{
    while(len--)
    {
        UART_PutChar(uratn, *buff);
        buff++;
    }
}



//-------------------------------------------------------------------------*
//������: UART_PutStr
//��  ��: �����ַ���(�� NULL ֹͣ����)
//��  ��: uratn:ģ�����磺UART0
//        str: ���͵ĵ�ַ��
//��  ��: ��
//��  ��: UART_PutStr (UART4, "123456789");ʵ�ʷ���9���ֽ�
//-------------------------------------------------------------------------*
void UART_PutStr(UART_Type * uratn, char *str)
{
    while(*str)
    {
        UART_PutChar(uratn, *str++);
    }
}

//-------------------------------------------------------------------------*
//������: UART_GetChar
//��  ��: �����ַ���(�� NULL ֹͣ����)
//��  ��: uratn:ģ�����磺UART0
//��  ��: �����յ�������
//��  ��: UART_GetChar (UART4); ��ȡ�����յ�������
//-------------------------------------------------------------------------*
char UART_GetChar(UART_Type * uratn)
{
     /* �ȴ��������� */
    while (!(UART_S1_REG(uratn) & UART_S1_RDRF_MASK));

    /* ��ȡ���յ���8λ���� */
    return UART_D_REG(uratn);
}


uint8_t UART_GetNum(UART_Type * uartn)
{
	if(uartn == UART0)
	{
		return 0;
	}
	if(uartn == UART1)
	{
		return 1;
	}
	if(uartn == UART2)
	{
		return 2;
	}
	if(uartn == UART3)
	{
		return 3;
	}
	if(uartn == UART4)
	{
		return 4;
	}
	return -1;
}

