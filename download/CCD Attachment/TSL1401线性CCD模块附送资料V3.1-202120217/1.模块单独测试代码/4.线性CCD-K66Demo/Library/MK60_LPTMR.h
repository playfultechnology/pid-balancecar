
#ifndef __LPTMR_H__
#define __LPTMR_H__

/**
 *  @brief LPTMR脉冲计数输入管脚选项
 */
typedef enum
{
    //只有1、2管脚，并没有0、3管脚
    LPT0_ALT1 = 1,      // PTA19
    LPT0_ALT2 = 2       // PTC5
} LPT0_ALTn;

/**
 *  @brief LPTMR脉冲计数触发方式
 */
typedef enum LPT_CFG
{
    LPT_Rising  = 0,    //上升沿触发
    LPT_Falling = 1     //下降沿触发
} LPT_CFG;

#define LPTMR_Flag_Clear()  (LPTMR0_CSR |= LPTMR_CSR_TCF_MASK)         //清除LPT比较标志位


void delayms(u16 ms);//ms延时

void delayus(u16 us);//us延时

void LPTMR_Delayms(uint16 ms);//ms延时

void LPTMR_delayus(uint16 us);//us延时

void LPTMR_PulseInit(LPT0_ALTn altn, uint16 count, LPT_CFG cfg);//LPTMR脉冲计数初始化

uint16_t LPTMR_PulseGet(void);//获取LPTMR脉冲计数值

void LPTMR_PulseClean(void);//清空LPTMR脉冲计数

void LPTMR_TimeStartms(void);//LPTMR计时函数ms

uint32_t LPTMR_TimeGetms(void);//获取LPTMR计时时间（ms）

void LPTMR_TimeStartus(void);//LPTMR计时函数us

uint32_t LPTMR_TimeGetus(void);//获取LPTMR计时时间（us）

void LPTMR_TimeClose();//关闭 LPTMR计时
#endif
