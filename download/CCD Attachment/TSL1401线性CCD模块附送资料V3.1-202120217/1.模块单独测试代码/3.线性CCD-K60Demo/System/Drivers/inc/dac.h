/**
  ******************************************************************************
  * @file    dac.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� dacģ������
  ******************************************************************************
  */
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"

//DAC ����ģʽѡ��
typedef enum
{
	DAC_TRIGER_MODE_NONE,
	DAC_TRIGER_MODE_SOFTWARE,
	DAC_TRIGER_MODE_HARDWARE,
}DAC_TriggerMode_TypeDef;
//���������
#define IS_DAC_TRIGGER_MODE(MODE) (((MODE) == DAC_TRIGER_MODE_NONE)     || \
                                   ((MODE) == DAC_TRIGER_MODE_SOFTWARE) || \
                                   ((MODE) == DAC_TRIGER_MODE_HARDWARE))

//���������														
#define IS_DAC_ALL_PERIPH(PERIPH)   ((PERIPH) == DAC0)


//DAC DMA�ж�Դ
#define DAC_DMAReq_DAC        (uint16_t)(0)
#define IS_DAC_DMAREQ(REQ)    ((REQ) == DAC_DMAReq_DAC)



//DAC�ж�Դ
#define DAC_IT_POINTER_BUTTOM   (uint16_t)(0)
#define DAC_IT_POINTER_TOP      (uint16_t)(1)
#define DAC_IT_WATER_MARK       (uint16_t)(2)
//���������
#define IS_DAC_IT(IT)    (((IT) == DAC_IT_POINTER_BUTTOM) || \
                          ((IT) == DAC_IT_POINTER_TOP)    || \
                          ((IT) == DAC_IT_WATER_MARK))
																 
//DAC ������ʹ��ģʽѡ��
typedef enum
{
	BUFFER_MODE_DISABLE,
	BUFFER_MODE_NORMAL,  
	BUFFER_MODE_SWING,    
	BUFFER_MODE_ONETIMESCAN,
}DAC_BufferMode_TypeDef;
#define IS_DAC_BUFFER_MODE(MODE) (((MODE) == BUFFER_MODE_DISABLE)     || \
                                  ((MODE) == BUFFER_MODE_NORMAL)      || \
                                  ((MODE) == BUFFER_MODE_SWING)       || \
                                  ((MODE) == BUFFER_MODE_ONETIMESCAN))

#define IS_DAC_BUFFER_UPPER_LIMIT(VALUE)  ((VALUE) <= 15)
//�ж�ˮλ���� 
typedef enum
{
	WATER_MODE_1WORD,
	WATER_MODE_2WORD,
	WATER_MODE_3WORD,
	WATER_MODE_4WORD,
}DAC_WaterMarkMode_TypeDef;
#define IS_DAC_WATERMARK_MODE(MODE) (((MODE) == WATER_MODE_1WORD) || \
                                     ((MODE) == WATER_MODE_2WORD) || \
                                     ((MODE) == WATER_MODE_3WORD) || \
                                     ((MODE) == WATER_MODE_4WORD))

//DAC ��ʼ���ṹ
typedef struct
{
	DAC_TriggerMode_TypeDef DAC_TrigerMode;        //������ʽ
	DAC_BufferMode_TypeDef DAC_BufferMode;         //��������ȡģʽ
	DAC_WaterMarkMode_TypeDef DAC_WaterMarkMode;  //ˮλ����
	uint8_t DAC_BufferUpperLimit;
	uint8_t DAC_BufferStartPostion;
}DAC_InitTypeDef;

//DAC���������ֵ����
#define IS_DAC_BUFFER_CNT(CNT) ((CNT) <= 16)
//DAC ���������ֵ
#define IS_DAC_BUFFER_VALUE(VALUE) ((VALUE) < 4096)
//��������ʵ�ֵĺ����ӿ�

void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct);
void DAC_DMACmd(DAC_Type* DACx, uint16_t DAC_DMAReq, FunctionalState NewState);
void DAC_ITConfig(DAC_Type* DACx, uint16_t DAC_IT, FunctionalState NewState);
ITStatus DAC_GetITStatus(DAC_Type* DACx, uint16_t DAC_IT);
void DAC_SoftwareTrigger(DAC_Type *DACx);
void DAC_SetBuffer(DAC_Type *DACx, uint16_t* DACBuffer,uint8_t NumberOfBuffer);
void DAC_SetValue(DAC_Type *DACx,uint16_t DAC_Value);

#ifdef __cplusplus
}
#endif

#endif

