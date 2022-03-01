
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

void delayms(u16 ms);

void LPTMR_Delayms(uint16 ms);

void LPTMR_PulseInit(LPT0_ALTn altn, uint16 count, LPT_CFG cfg);

uint16_t LPTMR_PulseGet(void);

void LPTMR_PulseClean(void);

void my_delay(uint32 time);

void LPTMR_TimeStartms(void);

uint32_t LPTMR_TimeGetms(void);

void LPTMR_TimeStartus(void);

uint32_t LPTMR_TimeGetus(void);

void LPTMR_TimeClose();
#endif
