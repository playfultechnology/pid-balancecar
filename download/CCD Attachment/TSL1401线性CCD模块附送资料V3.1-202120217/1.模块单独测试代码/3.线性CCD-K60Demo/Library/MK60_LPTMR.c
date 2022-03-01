
#include "include.h"
#include "MK60_LPTMR.h"

/**************************************************************************
*函数功能：ms延时函数
*入口参数：ms:延时ms数
*返 回 值：无
*实    例：delayms(50)==延时50ms
*注    意：使用延时的时候不能使用LPTMR别的功能
**************************************************************************/
void delayms(u16 ms)
{
  LPTMR_Delayms(ms);
}

/**************************************************************************
*函数功能：LPTMR脉冲计数初始化
*入口参数：LPT0_ALTn:LPTMR脉冲计数管脚
*          count    :LPTMR脉冲比较值
*          LPT_CFG  :LPTMR脉冲计数方式：上升沿计数或下降沿计数
*返 回 值：无
*实    例：LPTMR_pulse_init(LPT0_ALT1,32768,LPT_Rising)==A19,上升沿脉冲计数
*注    意：使用延时的时候不能使用LPTMR别的功能
**************************************************************************/
void LPTMR_PulseInit(LPT0_ALTn altn, uint16 count, LPT_CFG cfg)
{

    // 开启模块时钟
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

    //设置输入管脚
    if(altn == LPT0_ALT1)
    {
         PORTA_PCR19= PORT_PCR_MUX(6);
    }
    else if(altn == LPT0_ALT2)
    {
         PORTC_PCR5= PORT_PCR_MUX(4);
    }
    else                                    //不可能发生事件
    {
       ;
    }

    // 清状态寄存器
    LPTMR0_CSR = 0x00;                                          //先关了LPT，这样才能设置时钟分频,清空计数值等


    //选择时钟源
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //选择时钟源： 0 为 MCGIRCLK ，1为 LPO（1KHz） ，2为 ERCLK32K ，3为 OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //旁路 预分频/干扰滤波器 ,即不用 预分频/干扰滤波器(注释了表示使用预分频/干扰滤波器)
                      //| LPTMR_PSR_PRESCALE(1)           //预分频值 = 2^(n+1) ,n = 0~ 0xF
                    );


    // 设置累加计数值
    LPTMR0_CMR  =   LPTMR_CMR_COMPARE(count);                   //设置比较值
    LPTMR_Flag_Clear();
    // 管脚设置、使能中断
    LPTMR0_CSR  =  (0
                    | LPTMR_CSR_TPS(altn)       // 选择输入管脚 选择
                    | LPTMR_CSR_TMS_MASK        // 选择脉冲计数 (注释了表示时间计数模式)
                    | ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //脉冲计数器触发方式选择：0为高电平有效，上升沿加1
                    | LPTMR_CSR_TEN_MASK        //使能LPT(注释了表示禁用)
                    | LPTMR_CSR_TIE_MASK        //中断使能
                    //| LPTMR_CSR_TFC_MASK      //0:计数值等于比较值就复位；1：溢出复位（注释表示0）
                   );
}

/**************************************************************************
*函数功能：获取LPTMR脉冲计数值
*入口参数：无
*返 回 值：data：脉冲计数值
*实    例：无
**************************************************************************/
uint16_t LPTMR_PulseGet(void)
{
    uint16 data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//必须写入一个值才能正取读取
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //已经溢出了
    {

        data = ~0;                          //返回 0xffffffff 表示错误
    }
    else
    {
        data = LPTMR0_CNR;
    }
    return data;
}

/**************************************************************************
*函数功能：清空LPTMR脉冲计数
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void LPTMR_PulseClean(void)
{
    LPTMR0_CSR  &= ~LPTMR_CSR_TEN_MASK;     //禁用LPT的时候就会自动清计数器的值
    LPTMR0_CSR  |= LPTMR_CSR_TEN_MASK;
}

/**************************************************************************
*函数功能：LPTMR延时函数（ms）
*入口参数：ms：需要延时的ms数
*返 回 值：无
*实    例：LPTMR_delay_ms(50)==延时50ms
**************************************************************************/
void LPTMR_Delayms(uint16 ms)
{
    if(ms == 0)
    {
        return;
    }

    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;    //使能LPT模块时钟

    LPTMR0_CSR = 0x00;                      //先关了LPT，自动清计数器的值

    LPTMR0_CMR = ms;                        //设置比较值，即延时时间

    //选择时钟源
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //选择时钟源： 0 为 MCGIRCLK ，1为 LPO（1KHz） ，2为 ERCLK32K ，3为 OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //旁路 预分频/干扰滤波器 ,即不用 预分频/干扰滤波器(注释了表示使用预分频/干扰滤波器)
                      //| LPTMR_PSR_PRESCALE(1)           //预分频值 = 2^(n+1) ,n = 0~ 0xF
                    );

    //使能 LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // 选择输入管脚 选择
                    //| LPTMR_CSR_TMS_MASK      // 选择脉冲计数 (注释了表示时间计数模式)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //脉冲计数器触发方式选择：0为高电平有效，上升沿加1
                    | LPTMR_CSR_TEN_MASK        //使能LPT(注释了表示禁用)
                    //| LPTMR_CSR_TIE_MASK      //中断使能
                    //| LPTMR_CSR_TFC_MASK      //0:计数值等于比较值就复位；1：溢出复位（注释表示0）
                   );

    while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)); //等待比较值与计数值相等，即时间到了

    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;          //清除比较标志位

    return;
}

void LPTMR_TimeStartms(void)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;                        //使能LPT模块时钟

    LPTMR0_CSR = 0x00;                      //先关了LPT，自动清计数器的值

    LPTMR0_CMR = ~0;                        //设置比较值，即延时时间

    //选择时钟源
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //选择时钟源： 0 为 MCGIRCLK ，1为 LPO（1KHz） ，2为 ERCLK32K ，3为 OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //旁路 预分频/干扰滤波器 ,即不用 预分频/干扰滤波器(注释了表示使用预分频/干扰滤波器)
                      //| LPTMR_PSR_PRESCALE(1)           //预分频值 = 2^(n+1) ,n = 0~ 0xF
                    );

    //使能 LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // 选择输入管脚 选择
                    //| LPTMR_CSR_TMS_MASK      // 选择脉冲计数 (注释了表示时间计数模式)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //脉冲计数器触发方式选择：0为高电平有效，上升沿加1
                    | LPTMR_CSR_TEN_MASK        //使能LPT(注释了表示禁用)
                    //| LPTMR_CSR_TIE_MASK      //中断使能
                    //| LPTMR_CSR_TFC_MASK      //0:计数值等于比较值就复位；1：溢出复位（注释表示0）
                   );

}

/**************************************************************************
*函数功能：获取LPTMR计时时间（ms）
*入口参数：无
*返 回 值：data：计时时间~0==超时溢出，0~65535之间正常；
*实    例：无
**************************************************************************/
uint32_t LPTMR_TimeGetms(void)
{
    uint32 data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//必须写入一个值才能正取读取
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //已经溢出了
    {

        data = ~0;                          //返回 0xffffffff 表示错误
    }
    else
    {
        data = LPTMR0_CNR;                  //返回计时时间(此值最大为 0xffff)
    }

    return data;
}

/**************************************************************************
*函数功能：关闭 LPTMR计时
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void LPTMR_TimeClose()
{
    LPTMR0_CSR = 0x00; //先关了LPT，自动清计数器的值，清空溢出标记
}

/**************************************************************************
*函数功能：开启us计时
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void LPTMR_TimeStartus(void)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //使能 OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;                        //使能LPT模块时钟

    LPTMR0_CSR = 0x00;                                          //先关了LPT，自动清计数器的值

    LPTMR0_CMR = ~0;                                            //设置比较值为最大值

    //选择时钟源
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(3)          //选择时钟源： 0 为 MCGIRCLK ，1为 LPO（1KHz） ，2为 ERCLK32K ，3为 OSCERCLK
                      //| LPTMR_PSR_PBYP_MASK     //旁路 预分频/干扰滤波器 ,即不用 预分频/干扰滤波器(注释了表示使用预分频/干扰滤波器)
                      | LPTMR_PSR_PRESCALE(4)     //预分频值 = 2^(n+1) ,n = 0~ 0xF
                    );

    //使能 LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // 选择输入管脚 选择
                    //| LPTMR_CSR_TMS_MASK      // 选择脉冲计数 (注释了表示时间计数模式)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //脉冲计数器触发方式选择：0为高电平有效，上升沿加1
                    | LPTMR_CSR_TEN_MASK        //使能LPT(注释了表示禁用)
                    //| LPTMR_CSR_TIE_MASK      //中断使能
                    //| LPTMR_CSR_TFC_MASK      //0:计数值等于比较值就复位；1：溢出复位（注释表示0）
                   );
}

/**************************************************************************
*函数功能：获取LPTMR计时时间（us）
*入口参数：无
*返 回 值：data：计时时间~0==超时溢出，0~65535之间正常；
*实    例：无
**************************************************************************/
uint32_t LPTMR_TimeGetus(void)
{
    uint32_t data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//必须写入一个值才能正取读取
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //已经溢出了
    {
        data = ~0;                          //返回 0xffffffff 表示错误
    }
    else
    {
        data = (LPTMR0_CNR * 32) / 50; //进行单位换算
    }

    return data;
}
