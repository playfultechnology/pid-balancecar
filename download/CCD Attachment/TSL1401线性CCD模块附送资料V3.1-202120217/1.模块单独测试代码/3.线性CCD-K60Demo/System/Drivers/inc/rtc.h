/**
  ******************************************************************************
  * @file    rtc.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� ʵʱʱ������ ͷ�ļ�
  ******************************************************************************
  */
#ifndef __RTC_H_
#define	__RTC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"
//ʱ��ṹ��
typedef struct 
{
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;			
	uint8_t Month;
	uint8_t Date;
	uint8_t Week;	
	uint16_t Year;
	uint32_t TSRValue;
}RTC_CalanderTypeDef;		

//RTC �ж�Դ
#define RTC_IT_TAF    (uint16_t)(0)
#define RTC_IT_TOF    (uint16_t)(1)
#define RTC_IT_TIF    (uint16_t)(2)
#define IS_RTC_IT(IT)  (((IT) == RTC_IT_TAF) || \
                        ((IT) == RTC_IT_TOF) || \
                        ((IT) == RTC_IT_TIF))

//������ʵ�ֵĺ����ӿ��б�
void RTC_Init(void);
void RTC_SecondIntProcess(void);
void RTC_GetCalander(RTC_CalanderTypeDef * RTC_CalanderStruct);
uint8_t RTC_SetCalander(RTC_CalanderTypeDef * RTC_CalanderStruct);
void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState);


#ifdef __cplusplus
}
#endif

#endif


