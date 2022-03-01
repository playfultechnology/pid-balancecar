
#include "include.h"
#include "MK60_GPIO.h"

/* 定义五个指针数组保存 GPIOX 的地址 */
GPIO_MemMapPtr GPIOX[5] = {PTA_BASE_PTR, PTB_BASE_PTR, PTC_BASE_PTR, PTD_BASE_PTR, PTE_BASE_PTR};
PORT_MemMapPtr PORTX[5] = {PORTA_BASE_PTR, PORTB_BASE_PTR, PORTC_BASE_PTR, PORTD_BASE_PTR, PORTE_BASE_PTR};

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_Init
【功    能】初始化GPIO 并配置GPIO模式
【参    数】ptx_n ： 要初始化的GPIO， 在common.h中定义
【参    数】dir   ： GPIO方向和配置， 具体在GPIO.h中
【参    数】data  ： GPIO默认状态  1：高电平  0：低电平
【返 回 值】无
【实    例】GPIO_Init(PTA17, GPO, 1); //设置PTA17为输出模式 并设置为高电平
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinInit(PTXn_e ptx_n, GPIO_CFG dir, uint8_t data)
{

    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* 使能端口时钟 */
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK << ptx);

    /* 清除之前的复用功能 */
    PORTX[ptx]->PCR[ptn] &= ~(uint32)PORT_PCR_MUX_MASK;

    /* 设置复用功能为GPIO即普通IO口 */
    PORTX[ptx]->PCR[ptn] |= PORT_PCR_MUX(1);

    /* 配置GPIO模式 */
    PORTX[ptx]->PCR[ptn] |= dir;

    /* 设置GPIO方向 */
    if(dir)
    {
        GPIOX[ptx]->PDDR |= (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);
    }

    /* 设置端口默认状态 */
    if(data)
    {
        GPIOX[ptx]->PDOR |=  (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDOR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_PinSetDir
【功    能】设置IO口输入还是输出
【参    数】ptx_n ： IO口
【参    数】dir   ： GPIO方向   1：输出  0：输入
【返 回 值】无
【实    例】GPIO_PinSetDir(PTA17, 1); //设置PTA17输出
【注意事项】注意要使用GPIO初始化函数
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinSetDir(PTXn_e ptx_n, uint8_t dir)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* 设置GPIO方向 */
    if(dir)
    {
        GPIOX[ptx]->PDDR |= (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_PinWrite
【功    能】设置IO口输出
【参    数】ptx_n ： 要初始化的GPIO， 在common.h中定义
【参    数】data  ： GPIO输出状态  1：高电平  0：低电平
【返 回 值】无
【实    例】GPIO_PinWrite(PTA17, 1); //设置PTA17输出高电平
【注意事项】注意要使用GPIO初始化函数
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinWrite(PTXn_e ptx_n, uint8_t data)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* 设置端口默认状态 */
    if(data)
    {
        GPIOX[ptx]->PDOR |=  (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDOR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_PinRead
【功    能】读取IO口电平
【参    数】ptx_n ： 要初始化的GPIO， 在common.h中定义
【返 回 值】1： 高电平   0：低电平
【实    例】GPIO_PinRead(PTA17); //读取PTA17管脚电平
【注意事项】注意要使用GPIO初始化函数
--------------------------------------------------------------------------------------------------------*/
uint8_t GPIO_PinRead(PTXn_e ptx_n)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    return ( (GPIOX[ptx]->PDIR >> ptn) & 0x1 );
}

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_PinReverse
【功    能】GPIO取反
【参    数】ptx_n ： 要初始化的GPIO， 在common.h中定义
【返 回 值】无
【实    例】GPIO_PinReverse(PTA17); //取反PTA17管脚电平
【注意事项】注意要使用GPIO初始化函数
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinReverse(PTXn_e ptx_n)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    GPIOX[ptx]->PTOR ^= (1<<ptn);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************************************
                                                    gpio外部中断函数
**************************************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*------------------------------------------------------------------------------------------------------
【函    数】GPIO_PinReverse
【功    能】GPIO取反
【参    数】ptx_n ： 要初始化的GPIO， 在common.h中定义
【返 回 值】无
【实    例】GPIO_ExtiInit(PTA17, rising_down); //PTA17管脚上升沿触发中断
【注意事项】注意需要使用NVIC_SetPriority来配置PIT中断优先级   NVIC_EnableIRQ来使能中断
【注意事项】
【注意事项】优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
【注意事项】NVIC_SetPriority(PORTA_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
【注意事项】NVIC_EnableIRQ(PORTA_IRQn);			         //使能PORTA_IRQn的中断
--------------------------------------------------------------------------------------------------------*/
void GPIO_ExtiInit(PTXn_e ptx_n, exti_cfg cfg)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* 使能端口时钟 */
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK << ptx);

    //清除中断标志位
    PORTX[ptx]->ISFR = (uint32)1<<ptn;

    /* 配置端口功能 */
    PORT_PCR_REG(PORTX[ptx], ptn) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(cfg & 0x7f ) | PORT_PCR_PE_MASK | ((cfg & 0x80 ) >> 7);

    //设置端口为输入
    GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);

}
