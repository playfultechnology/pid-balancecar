#ifndef __CMT_h
#define __CMT_h


#define CMT_PRECISON 10000      //��ռ�ձ�
#define SERVO_INIT   750        //�����ֵ

void CMT_PwmInit(uint16_t period, uint16_t duty);//��ʼ��CMTģʽ

void CMT_PwmDuty(uint16_t duty);//����PWM��ռ�ձ�

#endif
