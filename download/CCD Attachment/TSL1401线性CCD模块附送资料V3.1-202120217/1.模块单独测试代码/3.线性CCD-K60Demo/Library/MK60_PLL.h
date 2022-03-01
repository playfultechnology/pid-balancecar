
#ifndef __PLL_H__
#define __PLL_H__

extern u8 core_clk;//单位MHZ
extern u8 bus_clk;//单位MHZ

//PLL参数
typedef enum clk_option
{
    PLLUSR      ,
    PLL48    = 48,
    PLL50    = 50,
    PLL96    = 96,
    PLL100   = 100,
    PLL110   = 110,
    PLL120   = 120,
    PLL125   = 125,
    PLL130   = 130,
    PLL140   = 140,
    PLL150   = 150,
    PLL160   = 160,
    PLL170   = 170,
    PLL180   = 180,
    PLL200   = 200, //推荐使主频为200或180
    PLL225   = 225, //超频到此频率，系统将不稳定，程序跑飞，慎用！
    PLL250   = 250
} clk_option;



/*********************** PLL功能函数 **************************/
void PLL_Init(clk_option);     //锁相环初始化




#endif
