#ifndef __MOTOR_H
#define __MOTOR_H


/*�����������*/
#define MOTOA_FTM  FTM0
#define MOTOB_FTM  FTM0
#define MOTOA_CH1  FTM_CH3//A���1 PC4
#define MOTOA_CH2  FTM_CH1//A���2 PC2
#define MOTOB_CH1  FTM_CH2//B���1 PC3
#define MOTOB_CH2  FTM_CH0//B���2 PC1




void Motor_init(void);

#endif