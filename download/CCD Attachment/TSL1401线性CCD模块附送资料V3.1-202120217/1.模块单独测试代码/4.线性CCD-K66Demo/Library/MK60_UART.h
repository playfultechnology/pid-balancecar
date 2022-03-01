
#ifndef __UART_H__
#define __UART_H__


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

#define UART4_RX    PTE25       //PTC14、PTE25
#define UART4_TX    PTE24       //PTC15、PTE24
/**********************************  UART(引脚复用) ***************************************/


void UART_Init (UART_Type * uratn, uint32 baud);//串口初始化

void UART_PutChar(UART_Type * uratn, char ch);//发送字节

/*-------------------------------------------------------------------------*
*函数名: UART_PutBuff
*功  能: 发送指定len个字节长度数组 （包括 NULL 也会发送）
*参  数: uratn:模块名如：UART0
*        buff: 发送的地址、
*        len : 发送指定长度
*返  回: 无
*简  例: UART_PutBuff (UART4, "123456789",5);实际发送5个字节‘1’‘2’‘3’‘4’‘5’
-------------------------------------------------------------------------*/
void UART_PutBuff(UART_Type * uratn, uint8_t *buff, uint32 len);//发送指定len个字节长度数组

void UART_PutStr(UART_Type * uratn, char *str);//发送字符串

char UART_GetChar(UART_Type * uratn);//获取串口收到的数据


uint8_t UART_GetNum(UART_Type * uartn);
/********************************************************************/

#endif
