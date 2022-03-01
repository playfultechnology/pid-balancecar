/**
  ******************************************************************************
  * @file    dma.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� DMA���� ͷ�ļ� 
  ******************************************************************************
  */
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "sys.h"

//=====================================
//      DMA request sources Number
//      ����DMA�����
//=====================================
#define UART0_REV_DMAREQ  2
#define UART0_TRAN_DMAREQ 3
#define UART1_REV_DMAREQ  4
#define UART1_TRAN_DMAREQ 5
#define UART2_REV_DMAREQ  6
#define UART2_TRAN_DMAREQ 7
#define UART3_REV_DMAREQ  8
#define UART3_TRAN_DMAREQ 9
#define UART4_REV_DMAREQ  10
#define UART4_TRAN_DMAREQ 11
#define UART5_REV_DMAREQ  12
#define UART5_TRAN_DMAREQ 13
#define I2S0_REV_DMAREQ   14
#define I2S0_TRAN_DMAREQ  15
#define SPI0_REV_DMAREQ   16
#define SPI0_TRAN_DMAREQ  17
#define SPI1_REV_DMAREQ   18
#define SPI1_TRAN_DMAREQ  19
#define I2C0_DMAREQ       22
#define I2C1_DMAREQ       23
#define FTM0_CH0_DMAREQ   24
#define FTM0_CH1_DMAREQ   25
#define FTM0_CH2_DMAREQ   26
#define FTM0_CH3_DMAREQ   27
#define FTM0_CH4_DMAREQ   28
#define FTM0_CH5_DMAREQ   29
#define FTM0_CH6_DMAREQ   30
#define FTM0_CH7_DMAREQ   31
#define FTM1_CH0_DMAREQ   32
#define FTM1_CH1_DMAREQ   33
#define FTM2_CH0_DMAREQ   34
#define FTM2_CH1_DMAREQ   35
#define FTM3_CH0_DMAREQ   36
#define FTM3_CH1_DMAREQ   37
#define FTM3_CH2_DMAREQ   38
#define FTM1_CH3_DMAREQ   39  
#define ADC0_DMAREQ       40
#define ADC1_DMAREQ       41
#define CMP0_DMAREQ       42
#define CMP1_DMAREQ       43
#define CMP2_DMAREQ       44
#define DAC0_DMAREQ       45
#define DAC1_DMAREQ       46
#define CMT_DMAREQ        47
#define PDB_DMAREQ        48
#define PORTA_DMAREQ      49
#define PORTB_DMAREQ      50
#define PORTC_DMAREQ      51
#define PORTD_DMAREQ      52
#define PORTE_DMAREQ      53
#define FTM3_CH4_DMAREQ   54
#define FTM3_CH5_DMAREQ   55
#define FTM3_CH6_DMAREQ   56
#define FTM3_CH7_DMAREQ   57
#define DMA_MUX1          58
#define DMA_MUX2          59
#define DMA_MUX3          60
//���������
#define IS_DMA_REQ(REQ)   (((REQ)>=2) && ((REQ)<= 60))
	
//DMA ��������ݳ���
#define DMA_SRC_8BIT   DMA_ATTR_SSIZE(0)
#define DMA_SRC_16BIT  DMA_ATTR_SSIZE(1)
#define DMA_SRC_32BIT  DMA_ATTR_SSIZE(2)


#define DMA_DST_8BIT   DMA_ATTR_DSIZE(0)
#define DMA_DST_16BIT  DMA_ATTR_DSIZE(1)
#define DMA_DST_32BIT  DMA_ATTR_DSIZE(2)
//���������
#define IS_DMA_ATTR_SSIZE(SIZE)   (((SIZE) == DMA_ATTR_SSIZE(0)) || \
                                   ((SIZE) == DMA_ATTR_SSIZE(1)) || \
																	 ((SIZE) == DMA_ATTR_SSIZE(2)))
																	 
#define IS_DMA_ATTR_DSIZE(SIZE)   (((SIZE) == DMA_ATTR_DSIZE(0)) || \
                                   ((SIZE) == DMA_ATTR_DSIZE(1)) || \
																	 ((SIZE) == DMA_ATTR_DSIZE(2)))
//DMAͨ���Ŷ���
#define  DMA_CH0    (uint8_t)(0)
#define  DMA_CH1    (uint8_t)(1)
#define  DMA_CH2    (uint8_t)(2)
#define  DMA_CH3    (uint8_t)(3)
#define  DMA_CH4    (uint8_t)(4)
#define  DMA_CH5    (uint8_t)(5)
#define  DMA_CH6    (uint8_t)(6)
#define  DMA_CH7    (uint8_t)(7)
#define  DMA_CH8    (uint8_t)(8)
#define  DMA_CH9    (uint8_t)(9)
#define  DMA_CH10   (uint8_t)(10)
#define  DMA_CH11   (uint8_t)(11)
#define  DMA_CH12   (uint8_t)(12)
#define  DMA_CH13   (uint8_t)(13)
#define  DMA_CH14   (uint8_t)(14)
#define  DMA_CH15   (uint8_t)(15)
//���������
#define IS_DMA_CH(CHL)  ((CHL) <= DMA_CH15)

//���������
#define IS_DMA_MINOR_LOOP(LEN) ((LEN) < 32767)

//DMA��ʼ���ṹ
typedef struct 
{
    uint8_t  Channelx;           //ͨ�����
    uint8_t  PeripheralDMAReq;   //�����DMA�����
    uint16_t MinorLoopLength;    //���ݳ���
    uint32_t TransferBytes;     //ÿһ��DMA����� ������ֽ���
    FunctionalState  DMAAutoClose;    //��ѭ������0ʱ �Ƿ�ر�DMA
                               //ENABLE �����Զ��ر�
                               //DISABLE �ر��Զ��ر�
    FunctionalState EnableState; //��ʼ�����Ƿ�������ʼ����
	
		//����Դ����
    uint32_t SourceBaseAddr ;    //Դ��ַ
    uint16_t SourceDataSize ;    //Դ��ַ��� 8 16 32
    uint8_t  SourceMinorInc;     //��ִ����һ������� ��ַ�Ƿ��ۼ�1  1�ۼ� 0 ���ۼ�
    uint32_t SourceMajorInc;     //����������� �Ƿ�ı�Դ��ַ
    //����Ŀ�ĵ�����
    uint32_t DestBaseAddr ;          //Ŀ���ַ
    uint16_t DestDataSize ;         //Ŀ���ַ���
    uint8_t  DestMinorInc;            //��ִ����һ������� ��ַ�Ƿ��ۼ�1  1�ۼ� 0 ���ۼ�
    uint32_t DestMajorInc;      //����������� �Ƿ�ı�Դ��ַ
}DMA_InitTypeDef;

//DMA �ж�Դ����
#define DMA_IT_HALF      (uint16_t)(0)
#define DMA_IT_MAJOR     (uint16_t)(1)
//���������
#define IS_DMA_IT(IT)  ((IT) == DMA_IT_HALF || (IT) == DMA_IT_MAJOR)


//������ʵ�ֵĽӿں����б�
void DMA_Init(DMA_InitTypeDef *DMA_InittStruct);
void DMA_SetEnableReq(uint8_t DMAChl,FunctionalState EnableState);
uint8_t DMA_IsComplete(uint8_t DMAChl);
void DMA_StructInit(DMA_InitTypeDef *DMA_InittStruct);
void DMA_SetCurrDataCounter(uint8_t DMAChl,uint16_t DataNumber);
uint16_t DMA_GetCurrDataCounter(uint8_t DMAChl);
void DMA_ClearITPendingBit(DMA_Type* DMAx, uint16_t DMA_IT, uint8_t DMA_CH);
void DMA_ITConfig(DMA_Type* DMAx, uint16_t DMA_IT, uint8_t DMA_CH, FunctionalState NewState);


#ifdef __cplusplus
}
#endif

#endif


