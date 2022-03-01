
#include "include.h"
#include "MK60_PIT.h"

/*------------------------------------------------------------------------------------------------------
【函    数】PIT_Init
【功    能】PIT定时器定时中断初始化
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【参    数】ms 中断时间，单位ms
【返 回 值】无
【实    例】pit_init(PIT0,1000); // PIT0中断，1000ms，即1s进入PIT0_interrupt()一次
【注意事项】注意需要使用NVIC_SetPriority来配置PIT中断优先级   NVIC_EnableIRQ来使能中断
【注意事项】
【注意事项】优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
【注意事项】NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
【注意事项】NVIC_EnableIRQ(PIT0_IRQn);			          //使能PIT0_IRQn的中断
--------------------------------------------------------------------------------------------------------*/
void PIT_Init(PITn pitn, uint32_t ms)
{
    //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

    /* 设置溢出中断时间 */
    PIT_LDVAL(pitn)  = ms * bus_clk * 1000;

    /* 清中断标志位 */
    PIT_Flag_Clear(pitn);

    /* 使能 PITn定时器,并开PITn中断 */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );

}


/*------------------------------------------------------------------------------------------------------
【函    数】PIT_delayms
【功    能】PIT定时器定时延时
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【参    数】ms  延时时间，单位ms
【返 回 值】无
【实    例】PIT_delayms(PIT0,1000); // PIT0延时，1000ms
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void PIT_Delayms(PITn pitn, uint32_t ms)
{
    //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* 设置溢出中断时间 */
    PIT_LDVAL(pitn)  = ms * bus_clk * 1000;

	/* 清中断标志位 */
    PIT_Flag_Clear(pitn);

    /* 清空计数器 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* 禁止PITn中断 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* 使能 PITn定时器 */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;

    /* 等待时间到 */
	while( !(PIT_TFLG(pitn) & PIT_TFLG_TIF_MASK) ){}

    /* 清中断标志位 */
    PIT_Flag_Clear(pitn);
}


/*------------------------------------------------------------------------------------------------------
【函    数】PIT_delayus
【功    能】PIT定时器定时延时
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【参    数】us  延时时间，单位us
【返 回 值】无
【实    例】PIT_delayms(PIT0,1000); // PIT0延时，1000us
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void PIT_Delayus(PITn pitn, uint32_t us)
{
    //PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* 设置溢出中断时间 */
    PIT_LDVAL(pitn)  = us * bus_clk;

	/* 清中断标志位 */
    PIT_Flag_Clear(pitn);

    /* 清空计数器 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* 禁止PITn中断 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* 使能 PITn定时器 */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;

    /* 等待时间到 */
	while( !(PIT_TFLG(pitn) & PIT_TFLG_TIF_MASK) ){}

    /* 清中断标志位 */
    PIT_Flag_Clear(pitn);
}


/*------------------------------------------------------------------------------------------------------
【函    数】PIT_TimeStart
【功    能】PIT定时器定时计时开始
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【返 回 值】无
【实    例】PIT_TimeStart(PIT0); // PIT0计时 当作程序计时器用
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void PIT_TimeStart(PITn pitn)
{
	//PIT 用的是 Bus Clock 总线频率

    /* 开启时钟*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* 使能PIT定时器时钟 ，调试模式下继续运行 */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* 设置溢出中断时间 */
    PIT_LDVAL(pitn)  = 0xFFFFFFFF;

	/* 清中断标志位 */
    PIT_Flag_Clear(pitn);

    /* 清空计数器 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* 禁止PITn中断 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* 使能 PITn定时器 */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;
}

/*------------------------------------------------------------------------------------------------------
【函    数】PIT_TimeGet
【功    能】PIT定时器得到计时时间
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【返 回 值】计时时间 单位us
【实    例】PIT_TimeGet(PIT0);  // PIT0计时 当作程序计时器用
【注意事项】注意要先用PIT_TimeStart(PIT0);开启计时器
【注意事项】注意PIT为32为定时器  最大计时 (0xFFFFFFFF / Bus Clock 总线频率) s
--------------------------------------------------------------------------------------------------------*/
uint32_t PIT_TimeGet(PITn pitn)
{
	return (0xFFFFFFFF - PIT_CVAL(pitn)) / bus_clk;
}

/*------------------------------------------------------------------------------------------------------
【函    数】PIT_TimeGet
【功    能】PIT定时器得到计时时间
【参    数】pitn:模块名PIT0或PIT1或PIT2或PIT3
【返 回 值】计时时间 单位us
【实    例】PIT_TimeGet(PIT0);  // PIT0计时 当作程序计时器用
【注意事项】注意要先用PIT_TimeStart(PIT0);开启计时器
【注意事项】注意PIT为32为定时器  最大计时 (0xFFFFFFFF / Bus Clock 总线频率) s
--------------------------------------------------------------------------------------------------------*/
void PIT_Close(PITn pitn)
{
	/* 清中断标志位 */
    PIT_Flag_Clear(pitn);

    /* 清空计数器 */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;
}
