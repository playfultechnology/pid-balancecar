#ifndef __MOTOR_H
#define __MOTOR_H


/*电机控制引脚*/
#define MOTOA_FTM  FTM0
#define MOTOB_FTM  FTM0
#define MOTOA_CH1  FTM_CH3//A电机1 PC4
#define MOTOA_CH2  FTM_CH1//A电机2 PC2
#define MOTOB_CH1  FTM_CH2//B电机1 PC3
#define MOTOB_CH2  FTM_CH0//B电机2 PC1




void Motor_init(void);

#endif