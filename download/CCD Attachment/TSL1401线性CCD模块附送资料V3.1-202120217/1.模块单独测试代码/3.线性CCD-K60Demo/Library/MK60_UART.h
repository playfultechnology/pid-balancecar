
#ifndef __UART_H__
#define __UART_H__



//定义模块号
typedef enum
{
    UART0,
    UART1,
    UART2,
    UART3,
    UART4,
    UART5,

    UART_MAX,
} UARTn_e;

extern volatile struct UART_MemMap *UARTN[UART_MAX];

/**********************************  UART(引脚复用) ***************************************/
//      模块通道    端口          可选范围                          建议
#define UART0_RX    PTA15       //PTA1、PTA15、PTB16、PTD6         PTA1不要用（与J-LINK冲突）
#define UART0_TX    PTA14        //PTA2、PTA14、PTB17、PTD7        PTA2不要用（与J-LINK冲突）

#define UART1_RX    PTC3        //PTC3、PTE1
#define UART1_TX    PTC4        //PTC4、PTE0

#define UART2_RX    PTD2        //PTD2
#define UART2_TX    PTD3        //PTD3

#define UART3_RX    PTE5       //PTB10、PTC16、PTE5
#define UART3_TX    PTE4       //PTB11、PTC17、PTE4

#define UART4_RX    PTC14       //PTC14、PTE25
#define UART4_TX    PTC15       //PTC15、PTE24

#define UART5_RX    PTE9        //PTD8、PTE9
#define UART5_TX    PTE8        //PTD9、PTE8
/**********************************  UART(引脚复用) ***************************************/


void UART_Init (UARTn_e uratn, uint32 baud);

void UART_PutChar(UARTn_e uratn, char ch);

void UART_PutBuff(UARTn_e uratn, s8 *buff, uint32 len);

void UART_PutStr(UARTn_e uratn, char *str);

char UART_GetChar(UARTn_e uratn);


#endif
