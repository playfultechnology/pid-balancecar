
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

void PIT_Init(PITn pitn, uint32_t ms);//PIT定时器定时中断初始化

void PIT_Delayms(PITn pitn, uint32_t ms);//PIT定时器定时延时

void PIT_Delayus(PITn pitn, uint32_t us);//PIT定时器定时延时

void PIT_TimeStart(PITn pitn);//PIT定时器定时计时开始

uint32_t PIT_TimeGet(PITn pitn);//PIT定时器得到计时时间

void PIT_Close(PITn pitn);//关闭PIT定时器

#endif