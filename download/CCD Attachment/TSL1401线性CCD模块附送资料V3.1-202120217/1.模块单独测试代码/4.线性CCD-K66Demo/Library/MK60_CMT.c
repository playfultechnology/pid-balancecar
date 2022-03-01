
#include "include.h"
#include "MK60_CMT.h"

/* 用于保存CMT计数器的周期计数值 内部使用 */
static uint16_t cmt_period;

/*------------------------------------------------------------------------------------------------------
【函    数】CMT_PwmInit
【功    能】初始化CMT模式
【参    数】freq  ： 设置PWM的频率
【参    数】duty  ： 设置PWM的占空比
【返 回 值】无
【实    例】CMT_PwmInit(50, 50); //频率50HZ，占空比为百分之（50/CMT_PRECISON *100）;
【注意事项】CMT输出PWM只能用PD7管脚
--------------------------------------------------------------------------------------------------------*/
void CMT_PwmInit(uint16_t freq, uint16_t duty)
{
    uint32_t temp_clk;
    uint32_t temp_high_num, temp_low_num;
    uint32_t temp_div;

    SIM->SCGC4 |= SIM_SCGC4_CMT_MASK;//使能时钟
    PORTD_PCR7= PORT_PCR_MUX(2); //设置复用功能为CMT功能
    CMT->OC |= CMT_OC_IROPEN_MASK;//使能输出

    temp_clk = bus_clk*1000*1000/8;//CMT模块固定八分频
    temp_div = temp_clk/freq;
    temp_div = (temp_div>>16);
    if(temp_div>0x0f)   temp_div = 0x0f; //计算最佳分频

    CMT->PPS = CMT_PPS_PPSDIV(temp_div);//设置分频
    cmt_period = temp_clk/(temp_div+1)/freq;//计算一个周期需要计数的次数

    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);//计算高低电平的计数次数
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);

    temp_low_num--;                     //设置低电平时间
    CMT->CMD1 = temp_low_num >> 8;      //设置低电平时间
    CMT->CMD2 = (uint8)temp_low_num;    //设置低电平时间
    CMT->CMD3 = temp_high_num >> 8;     //设置高电平时间
    CMT->CMD4 = (uint8)temp_high_num;   //设置高电平时间

    CMT->MSC = CMT_MSC_BASE_MASK | CMT_MSC_MCGEN_MASK;//设置模式且使能CMT模块
}


/*------------------------------------------------------------------------------------------------------
【函    数】CMT_PwmDuty
【功    能】初始化CMT模式
【参    数】duty  ： 设置PWM的占空比
【返 回 值】无
【实    例】CMT_PwmInit(50); //占空比为百分之（50/CMT_PRECISON *100）;
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void CMT_PwmDuty(uint16_t duty)
{

    uint32_t temp_high_num, temp_low_num;

    //计算高低电平的计数次数
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);

    //设置低电平时间
    temp_low_num--;
    CMT->CMD1 = temp_low_num >> 8;
    CMT->CMD2 = (uint8)temp_low_num;

    //设置高电平时间
    CMT->CMD3 = temp_high_num >> 8;
    CMT->CMD4 = (uint8)temp_high_num;

}


