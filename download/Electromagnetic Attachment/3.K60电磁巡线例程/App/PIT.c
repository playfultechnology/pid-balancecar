
#include "include.h"
#include "PIT.h"


/*------------------------------------------------------------------------------------------------------
【函    数】MyPIT_Init
【功    能】初始化PIT定时器
【参    数】无
【返 回 值】无
【实    例】MyPIT_Init(); //PIT0定时器初始化
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void MyPIT_Init(void)
{
    PIT_Init(PIT1,   5);       //PIT1,5MS
    NVIC_SetPriority(PIT1_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
    /* 优先级配置 抢占优先级0  子优先级2   越小优先级越高  抢占优先级可打断别的中断 */

    NVIC_EnableIRQ(PIT1_IRQn);			                  //使能PIT0_IRQn的中断
}


