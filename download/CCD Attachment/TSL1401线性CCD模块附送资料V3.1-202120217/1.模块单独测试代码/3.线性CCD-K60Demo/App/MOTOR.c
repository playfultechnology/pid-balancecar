
#include "include.h"
#include "motor.h"

static uint8 Init_Duty=0;//初始化占空比0
static uint16 PWM_Freq=10000;//PWM频率10KHZ


/*****************初始化电机控制引脚*****************/
void Motor_init(void)
{
  FTM_PwmInit(MOTOA_FTM,MOTOA_CH1,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOA_FTM,MOTOA_CH2,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOB_FTM,MOTOB_CH1,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOB_FTM,MOTOB_CH2,PWM_Freq,Init_Duty);
}