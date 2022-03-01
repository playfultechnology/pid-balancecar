
#ifndef __UART_H__
#define __UART_H__


/**********************************  UART(���Ÿ���) ***************************************/
//      ģ��ͨ��    �˿�          ��ѡ��Χ                          ����
#define UART0_RX    PTA15       //PTA1��PTA15��PTB16��PTD6         PTA1��Ҫ�ã���J-LINK��ͻ��
#define UART0_TX    PTA14        //PTA2��PTA14��PTB17��PTD7        PTA2��Ҫ�ã���J-LINK��ͻ��

#define UART1_RX    PTC3        //PTC3��PTE1
#define UART1_TX    PTC4        //PTC4��PTE0

#define UART2_RX    PTD2        //PTD2
#define UART2_TX    PTD3        //PTD3

#define UART3_RX    PTE5       //PTB10��PTC16��PTE5
#define UART3_TX    PTE4       //PTB11��PTC17��PTE4

#define UART4_RX    PTE25       //PTC14��PTE25
#define UART4_TX    PTE24       //PTC15��PTE24
/**********************************  UART(���Ÿ���) ***************************************/


void UART_Init (UART_Type * uratn, uint32 baud);//���ڳ�ʼ��

void UART_PutChar(UART_Type * uratn, char ch);//�����ֽ�

/*-------------------------------------------------------------------------*
*������: UART_PutBuff
*��  ��: ����ָ��len���ֽڳ������� ������ NULL Ҳ�ᷢ�ͣ�
*��  ��: uratn:ģ�����磺UART0
*        buff: ���͵ĵ�ַ��
*        len : ����ָ������
*��  ��: ��
*��  ��: UART_PutBuff (UART4, "123456789",5);ʵ�ʷ���5���ֽڡ�1����2����3����4����5��
-------------------------------------------------------------------------*/
void UART_PutBuff(UART_Type * uratn, uint8_t *buff, uint32 len);//����ָ��len���ֽڳ�������

void UART_PutStr(UART_Type * uratn, char *str);//�����ַ���

char UART_GetChar(UART_Type * uratn);//��ȡ�����յ�������


uint8_t UART_GetNum(UART_Type * uartn);
/********************************************************************/

#endif
