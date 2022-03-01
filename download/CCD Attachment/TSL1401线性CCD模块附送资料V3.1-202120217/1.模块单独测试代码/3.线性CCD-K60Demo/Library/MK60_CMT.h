#ifndef __CMT_h
#define __CMT_h

#define CMT_PRECISON 10000u
#define SERVO_INIT   750

void CMT_PwmInit(uint16_t period, uint16_t duty);

void CMT_PwmDuty(uint16_t duty);

#endif
