
#include "include.h"
#include "MK60_UART.h"

/* 定义6个指针数组保存 UARTn_e 的地址 */
UART_MemMapPtr UARTN[5] = {UART0_BASE_PTR, UART1_BASE_PTR, UART2_BASE_PTR, UART3_BASE_PTR, UART4_BASE_PTR};


//加入以下代码,支持printf函数,而不需要选择use MicroLIB
//IAR，需要在options -> C/C++compiler -> Preprocessor 添加宏定义 _DLIB_FILE_DESCRIPTOR
#if 1
//#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
//重定义fputc函数  使用串口4作为printf的打印口
int fputc(int ch, FILE *f)
{
    while(!(UART_S1_REG(UARTN[3]) & UART_S1_TDRE_MASK));
    //发送数据
    UART_D_REG(UARTN[3]) = (uint8)ch;
	return ch;
}
#endif


//-------------------------------------------------------------------------*
//函数名: UART_Init
//功  能: 初始化UART
//参  数: uratn:模块名如：UART0
//        baud: 波特率
//返  回: 无
//简  例: uart_init(UART4,115200);UART4对应引脚参考UART.H文件
//-------------------------------------------------------------------------*
void UART_Init (UART_Type * uratn, uint32 baud)
{
    register uint16 sbr, brfa;
    uint8 temp;
    uint32 sysclk;     //时钟

	uint8_t uart_num = UART_GetNum(uratn);
    /* 配置 UART功能的 复用管脚 */
    switch(uart_num)
    {
    case 0:
        SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;      //使能 UART0 时钟

        if(UART0_RX == PTA1)
        {
            PORTA_PCR1= PORT_PCR_MUX(2);       //使能PTA1引脚第二功能，即UART0_RXD
        }
        else if(UART0_RX == PTA15)
        {
             PORTA_PCR15= PORT_PCR_MUX(3);       //使能PTA15引脚第3功能
        }
        else if(UART0_RX == PTB16)
        {
             PORTB_PCR16= PORT_PCR_MUX(3);       //使能PTB16引脚第3功能
        }
        else if(UART0_RX == PTD6)
        {
             PORTD_PCR6= PORT_PCR_MUX(3);       //使能PTD6引脚第3功能
        }
        else
        {
            break;
        }

        if(UART0_TX == PTA2)
        {
             PORTA_PCR2= PORT_PCR_MUX(2);       //使能PTA2引脚第二功能
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

    //设置的时候，应该禁止发送接受
    UART_C2_REG(uratn) &= ~(0
                                   | UART_C2_TE_MASK
                                   | UART_C2_RE_MASK
                                  );


    //配置成8位无校验模式
    //设置 UART 数据格式、校验方式和停止位位数。通过设置 UART 模块控制寄存器 C1 实现；
    UART_C1_REG(uratn) |= (0
                                  //| UART_C2_M_MASK                    //9 位或 8 位模式选择 : 0 为 8位 ，1 为 9位（注释了表示0，即8位） （如果是9位，位8在UARTx_C3里）
                                  //| UART_C2_PE_MASK                   //奇偶校验使能（注释了表示禁用）
                                  //| UART_C2_PT_MASK                   //校验位类型 : 0 为 偶校验 ，1 为 奇校验
                                 );

    //计算波特率，串口0、1使用内核时钟，其它串口使用bus时钟
    if ((uratn == UART0) || (uratn == UART1))
    {
        sysclk = core_clk * 1000*1000;                                   //内核时钟
    }
    else
    {
        sysclk =  bus_clk * 1000*1000;                                    //bus时钟(是内核时钟的一半)
    }

    //UART 波特率 = UART 模块时钟 / (16 × (SBR[12:0] + BRFA))
    //不考虑 BRFA 的情况下， SBR = UART 模块时钟 / (16 * UART 波特率)
    sbr = (uint16)(sysclk / (baud * 16));
    if(sbr > 0x1FFF)sbr = 0x1FFF;                                       //SBR 是 13bit，最大为 0x1FFF

    //已知 SBR ，则 BRFA =  = UART 模块时钟 / UART 波特率 - 16 ×SBR[12:0]
    brfa = (sysclk / baud)  - (sbr * 16);


    //写 SBR
    temp = (uint16)(UART_BDH_REG(uratn)) & (uint16)(~UART_BDH_SBR_MASK);           //缓存 清空 SBR 的 UARTx_BDH的值
    UART_BDH_REG(uratn) = temp |  UART_BDH_SBR(sbr >> 8);        //先写入SBR高位
    UART_BDL_REG(uratn) = UART_BDL_SBR(sbr);                     //再写入SBR低位

    //写 BRFD
    temp =(uint16)( UART_C4_REG(uratn)) & (uint16)(~UART_C4_BRFA_MASK) ;           //缓存 清空 BRFA 的 UARTx_C4 的值
    UART_C4_REG(uratn) = temp |  UART_C4_BRFA(brfa);             //写入BRFA



    //设置FIFO(FIFO的深度是由硬件决定的，软件不能设置)
    UART_PFIFO_REG(uratn) |= (0
                                     | UART_PFIFO_TXFE_MASK               //使能TX FIFO(注释表示禁止)
                                     //| UART_PFIFO_TXFIFOSIZE(0)         //（只读）TX FIFO 大小，0为1字节，1~6为 2^(n+1)字节
                                     | UART_PFIFO_RXFE_MASK               //使能RX FIFO(注释表示禁止)
                                     //| UART_PFIFO_RXFIFOSIZE(0)         //（只读）RX FIFO 大小，0为1字节，1~6为 2^(n+1)字节
                                    );

    /* 允许发送和接收 */
    UART_C2_REG(uratn) |= (0
                                  | UART_C2_TE_MASK                     //发送使能
                                  | UART_C2_RE_MASK                     //接收使能
                                  //| UART_C2_TIE_MASK                  //发送中断或DMA传输请求使能（注释了表示禁用）
                                  //| UART_C2_TCIE_MASK                 //发送完成中断使能（注释了表示禁用）
                                  | UART_C2_RIE_MASK                    //接收满中断或DMA传输请求使能（注释了表示禁用）
                                 );


}



//-------------------------------------------------------------------------*
//函数名: UART_PutChar
//功  能: 发送一个字节
//参  数: uratn:模块名如：UART0
//         ch: 发送的字节
//返  回: 无
//简  例: UART_PutChar (UART4, 0x66);
//-------------------------------------------------------------------------*
void UART_PutChar(UART_Type * uratn, char ch)
{
    //等待发送缓冲区空
    while(!(UART_S1_REG(uratn) & UART_S1_TDRE_MASK));
    //发送数据
    UART_D_REG(uratn) = (uint8)ch;
}


//-------------------------------------------------------------------------*
//函数名: UART_PutBuff
//功  能: 发送指定len个字节长度数组 （包括 NULL 也会发送）
//参  数: uratn:模块名如：UART0
//        buff: 发送的地址、
//        len : 发送指定长度
//返  回: 无
//简  例: UART_PutBuff (UART4, "123456789",5);实际发送5个字节‘1’‘2’‘3’‘4’‘5’
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
//函数名: UART_PutStr
//功  能: 发送字符串(遇 NULL 停止发送)
//参  数: uratn:模块名如：UART0
//        str: 发送的地址、
//返  回: 无
//简  例: UART_PutStr (UART4, "123456789");实际发送9个字节
//-------------------------------------------------------------------------*
void UART_PutStr(UART_Type * uratn, char *str)
{
    while(*str)
    {
        UART_PutChar(uratn, *str++);
    }
}

//-------------------------------------------------------------------------*
//函数名: UART_GetChar
//功  能: 发送字符串(遇 NULL 停止发送)
//参  数: uratn:模块名如：UART0
//返  回: 串口收到的数据
//简  例: UART_GetChar (UART4); 获取串口收到的数据
//-------------------------------------------------------------------------*
char UART_GetChar(UART_Type * uratn)
{
     /* 等待接收满了 */
    while (!(UART_S1_REG(uratn) & UART_S1_RDRF_MASK));

    /* 获取接收到的8位数据 */
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

