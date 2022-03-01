
#ifndef _PIT_H_
#define _PIT_H_

#define PIT_Flag_Clear(PITn)   PIT_TFLG(PITn)|=PIT_TFLG_TIF_MASK      //清中断标志

//模块定义
typedef enum PITn
{
    PIT0,
    PIT1,
    PIT2,
    PIT3
} PITn;


void PIT_Init(PITn pitn, uint32_t ms);

void PIT_Delayms(PITn pitn, uint32_t ms);

void PIT_Delayus(PITn pitn, uint32_t us);

void PIT_TimeStart(PITn pitn);

uint32_t PIT_TimeGet(PITn pitn);

void PIT_Close(PITn pitn);

#endif