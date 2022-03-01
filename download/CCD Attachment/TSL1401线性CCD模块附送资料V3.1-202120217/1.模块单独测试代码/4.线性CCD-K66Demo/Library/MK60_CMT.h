#ifndef __CMT_h
#define __CMT_h


#define CMT_PRECISON 10000      //满占空比
#define SERVO_INIT   750        //舵机中值

void CMT_PwmInit(uint16_t period, uint16_t duty);//初始化CMT模式

void CMT_PwmDuty(uint16_t duty);//设置PWM的占空比

#endif
