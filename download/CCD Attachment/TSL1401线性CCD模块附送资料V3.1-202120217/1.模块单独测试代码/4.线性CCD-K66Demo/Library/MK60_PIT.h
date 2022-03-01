
#ifndef _PIT_H_
#define _PIT_H_

#define PIT_Flag_Clear(PITn)   PIT_TFLG(PITn)|=PIT_TFLG_TIF_MASK      //���жϱ�־

//ģ�鶨��
typedef enum PITn
{
    PIT0,
    PIT1,
    PIT2,
    PIT3
} PITn;

void PIT_Init(PITn pitn, uint32_t ms);//PIT��ʱ����ʱ�жϳ�ʼ��

void PIT_Delayms(PITn pitn, uint32_t ms);//PIT��ʱ����ʱ��ʱ

void PIT_Delayus(PITn pitn, uint32_t us);//PIT��ʱ����ʱ��ʱ

void PIT_TimeStart(PITn pitn);//PIT��ʱ����ʱ��ʱ��ʼ

uint32_t PIT_TimeGet(PITn pitn);//PIT��ʱ���õ���ʱʱ��

void PIT_Close(PITn pitn);//�ر�PIT��ʱ��

#endif