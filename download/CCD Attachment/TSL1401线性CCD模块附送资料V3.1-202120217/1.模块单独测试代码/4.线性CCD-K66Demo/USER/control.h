#ifndef __CONTROL_H
#define __CONTROL_H
#include "include.h"



#define PI 3.14159265
#define ZHONGZHI 0


void Kinematic_Analysis(float velocity,float angle);

void TIM2_IRQHandler(void) ;

void Set_Pwm(int motor_a,int motor_b,int servo);

void Key(void);

void Xianfu_Pwm(void);

u8 Turn_Off( int voltage);

void Get_Angle(u8 way);


int Incremental_PI_A (int Encoder,int Target);

int Incremental_PI_B (int Encoder,int Target);

void Get_RC(void);





#endif
