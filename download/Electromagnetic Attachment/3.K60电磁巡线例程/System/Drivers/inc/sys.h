/**
  ******************************************************************************
  * @file    sys.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� ϵͳ��API����ͷ�ļ�
  ******************************************************************************
  */
	 
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "MK60DZ10.h"
//���ú�
//#define USE_FULL_ASSERT                          //�Ƿ��������������ƣ����������Ҫ�û�ʵ�� assert_failed����
//#define DEBUG_PRINT      1                           //�Ƿ��ô��ڴ�ӡ������Ϣ
//#define SYSTEM_SUPPORT_OS				               //֧��uCOS����ϵͳ

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))
typedef enum{FALSE = 0, TRUE = !FALSE} ErrorState;

#ifndef NULL
	#define NULL 0
#endif

/* CH_Kinetis version information */
#define CHK_VERSION                      2L              /**< major version number */
#define CHK_SUBVERSION                4L              /**< minor version number */
#define CHK_REVISION                     1L              /**< revise version number */

/* CH_Kinetis version */
#define FW_VERSION                ((CHK_VERSION * 10000) + \
                                         (CHK_SUBVERSION * 100) + CHK_REVISION)
//���������
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */




//��������Ϣ����Ƶ��Ϣ�ṹ
typedef struct
{
	uint8_t FamilyType;    //Kinetisϵ�м������ͺ� 
	uint8_t ResetState;    //��λԭ��
	uint8_t SiliconRev;    //SiliconRev
	uint16_t PinCnt;       //��������
	uint32_t PFlashSize;   //PFlash��С
	uint32_t FlexNVMSize;  //FlexNVM��С
	uint32_t RAMSize;      //RAM��С
	uint32_t CoreClock;    //�ں�ʱ��
	uint32_t BusClock;     //����ʱ��
	uint32_t FlexBusClock; //FlexBusʱ��
	uint32_t FlashClock;   //Flashʱ��
} CPUInfoType_t;
extern CPUInfoType_t CPUInfo;

//MCGʱ��Դѡ��
#define ClockSource_IRC        ((uint8_t)0x0)
#define ClockSource_EX50M   ((uint8_t)0x5)
#define ClockSource_EX8M     ((uint8_t)0x6)
//�����������
#define IS_CLOCK_OPTION(SOURCE)			 (((SOURCE)  == ClockSource_IRC)  ||  \
                                       ((SOURCE) == ClockSource_EX50M) || \
                                       ((SOURCE) == ClockSource_EX8M))
//MCGʱ�����ѡ��
#define CoreClock_200M      ((uint8_t)0x13) //���100MƵ�� 200M����ΪС����Ƶĳ�Ƶ
#define CoreClock_100M			((uint8_t)0x12)
#define CoreClock_96M				((uint8_t)0x11)
#define CoreClock_72M				((uint8_t)0x10)
#define CoreClock_64M				((uint8_t)0x9)
#define CoreClock_48M				((uint8_t)0x8)
//��������
#define IS_CLOCK_SELECT(CLOCK_TYPE)   (((CLOCK_TYPE) ==    CoreClock_100M)||   \
																			 ((CLOCK_TYPE) ==    CoreClock_200M)||   \
																			 ((CLOCK_TYPE) ==    CoreClock_96M) ||   \
																			 ((CLOCK_TYPE) ==    CoreClock_72M) ||   \
																			 ((CLOCK_TYPE) ==    CoreClock_64M) ||   \
																			 ((CLOCK_TYPE) ==    CoreClock_48M))
//NVIC�жϷ���ѡ��
#define NVIC_PriorityGroup_0         ((uint32_t)0x7) /*!< 0 bits for pre-emption priority   4 bits for subpriority */                                               
#define NVIC_PriorityGroup_1         ((uint32_t)0x6) /*!< 1 bits for pre-emption priority   3 bits for subpriority */                                                  
#define NVIC_PriorityGroup_2         ((uint32_t)0x5) /*!< 2 bits for pre-emption priority   2 bits for subpriority */                                                   
#define NVIC_PriorityGroup_3         ((uint32_t)0x4) /*!< 3 bits for pre-emption priority   1 bits for subpriority */                                                   
#define NVIC_PriorityGroup_4         ((uint32_t)0x3) /*!< 4 bits for pre-emption priority   0 bits for subpriority */
//�����������
#define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PriorityGroup_0) || \
                                       ((GROUP) == NVIC_PriorityGroup_1) || \
                                       ((GROUP) == NVIC_PriorityGroup_2) || \
                                       ((GROUP) == NVIC_PriorityGroup_3) || \
                                       ((GROUP) == NVIC_PriorityGroup_4))
#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)
#define IS_NVIC_SUB_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

//VETOR_OFFSET����
#define IS_VECTOR_OFFSET(OFFSET)  ((OFFSET) % 4 == 0)

//������ʵ�ֵĽӿں����б�
void SystemClockSetup(uint8_t ClockOption,uint16_t CoreClock);  //����ϵͳʱ��
void SystemSoftReset(void);                                     //��λ
void GetCPUInfo(void);                                          //��ô�������Ϣ
void EnableInterrupts(void);                                    //ʱ�����ж�
void DisableInterrupts(void);                                   //�ر����ж�
void SetVectorTable(uint32_t offset);                           //�����ж�������ʼλ��
void NVIC_EnableIRQ(IRQn_Type IRQn);                            //�����ж�                  
void NVIC_DisableIRQ(IRQn_Type IRQn);                           //�ر��ж�
void NVIC_Init(IRQn_Type IRQn,uint32_t PriorityGroup,uint32_t PreemptPriority,uint32_t SubPriority); //�����ж����ȼ�
uint16_t GetFWVersion(void);

#ifdef __cplusplus
}
#endif

#endif

