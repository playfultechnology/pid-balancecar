
#ifndef __PLL_H__
#define __PLL_H__

extern uint8_t core_clk;//单位MHZ
extern uint8_t bus_clk; //单位MHZ

//PLL参数
typedef enum clk_option
{
    PLLUNULL ,
    PLL80    ,
    PLL90    ,
    PLL100   ,
    PLL120   ,
    PLL130   ,
    PLL140   ,
    PLL150   ,
    PLL160   ,
    PLL170   ,
    PLL180   ,
    PLL200   ,
    PLL220   ,
    PLL225   ,
    PLL230   ,
    PLL235   ,
    PLL237_5 , //237.5M
} clk_option;

/*********************** PLL功能函数 **************************/
void PLL_Init(clk_option);     //锁相环初始化




#endif
