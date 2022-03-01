
#include "include.h"
#include "motor.h"

static uint8 Init_Duty=0;//��ʼ��ռ�ձ�0
static uint16 PWM_Freq=10000;//PWMƵ��10KHZ


/*****************��ʼ�������������*****************/
void Motor_init(void)
{
  FTM_PwmInit(MOTOA_FTM,MOTOA_CH1,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOA_FTM,MOTOA_CH2,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOB_FTM,MOTOB_CH1,PWM_Freq,Init_Duty);
  FTM_PwmInit(MOTOB_FTM,MOTOB_CH2,PWM_Freq,Init_Duty);
}