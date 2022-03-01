/**
  ******************************************************************************
  * @file    can.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� CAN �����ļ�
  ******************************************************************************
  */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"

//CAN��ʼ���ṹ
typedef struct
{
	uint32_t CANxMap;               //���ݶ˿ڶ���
	uint32_t CAN_BaudRateSelect;    //���ʶ���
	FunctionalState FilterEnable;   //�Ƿ�ʹ��������˹��� 
}CAN_InitTypeDef;

//���������														
#define IS_CAN_ALL_PERIPH(PERIPH)  ((PERIPH) == CAN0)  || \
                                   ((PERIPH) == CAN1)

//CAN�ṹ����
typedef struct
{
    uint32_t CAN_Index:2;
	  uint32_t CAN_Alt_Index:3;
    uint32_t CAN_GPIO_Index:3;
    uint32_t CAN_TX_Pin_Index:6;
		uint32_t CAN_RX_Pin_Index:6;
}CAN_MapTypeDef;

//����������Ϣ��Ϣ�ṹ 
typedef struct
{
  uint32_t Id;      //ID��
	uint32_t MBIndex; //ʹ�õ������ ���ͺͽ��ղ����ظ�
	uint8_t IDE;      //��չ֡��־
  uint8_t RTR;      //Զ��֡��־
  uint8_t DLC;      //����֡����
  uint8_t Data[8];  //֡���� �8�ֽ�
} CAN_TxMsgTypeDef;

//����������Ϣ�ṹ
typedef struct
{
  uint32_t Id;      //ID��
	uint32_t MBIndex; //ʹ�õ������ ���ͺͽ��ղ����ظ�
  uint8_t IDE;      //��չ֡��־
  uint8_t RTR;      //Զ��֡��־
  uint8_t DLC;      //����֡����
  uint8_t Data[8];  //֡���� �8�ֽ�
} CAN_RxMsgTypeDef;

//֡���� 
#define CAN_DLC_1Byte    (uint8_t)(1)
#define CAN_DLC_2Byte    (uint8_t)(2)
#define CAN_DLC_3Byte    (uint8_t)(3)
#define CAN_DLC_4Byte    (uint8_t)(4)
#define CAN_DLC_5Byte    (uint8_t)(5)
#define CAN_DLC_6Byte    (uint8_t)(6)
#define CAN_DLC_7Byte    (uint8_t)(7)
#define CAN_DLC_8Byte    (uint8_t)(8)
#define IS_CAN_DLC(DLC)  (((DLC) == CAN_DLC_1Byte) || \
											    ((DLC) == CAN_DLC_2Byte) || \
											    ((DLC) == CAN_DLC_3Byte) || \
											    ((DLC) == CAN_DLC_4Byte) || \
											    ((DLC) == CAN_DLC_5Byte) || \
											    ((DLC) == CAN_DLC_6Byte) || \
											    ((DLC) == CAN_DLC_7Byte) || \
											    ((DLC) == CAN_DLC_8Byte))



//���Ŷ���
#define CAN0_TX_PA12_RX_PA13   (0x00034c08U)
#define CAN0_TX_PB18_RX_PB19   (0x0004d228U)
#define CAN1_TX_PE24_RX_PE25   (0x00065889U)
#define CAN1_TX_PC17_RX_PC16   (0x00041149U)
#define IS_CAN_MAP(MAP)       (((MAP) == CAN0_TX_PA12_RX_PA13) || \
                               ((MAP) == CAN0_TX_PB18_RX_PB19) || \
                               ((MAP) == CAN1_TX_PE24_RX_PE25) || \
                               ((MAP) == CAN1_TX_PC17_RX_PC16))

//����Ŷ���
#define CAN_MB0                (uint32_t)(0)
#define CAN_MB1                (uint32_t)(1)
#define CAN_MB2                (uint32_t)(2)
#define CAN_MB3                (uint32_t)(3)
#define CAN_MB4                (uint32_t)(4)
#define CAN_MB5                (uint32_t)(5)
#define CAN_MB6                (uint32_t)(6)
#define CAN_MB7                (uint32_t)(7)
#define CAN_MB8                (uint32_t)(8)
#define CAN_MB9                (uint32_t)(9)
#define CAN_MB10               (uint32_t)(10)
#define CAN_MB11               (uint32_t)(11)
#define CAN_MB12               (uint32_t)(12)
#define CAN_MB13               (uint32_t)(13)
#define CAN_MB14               (uint32_t)(14)
#define CAN_MB15               (uint32_t)(15)
#define IS_CAN_MB_NUM(NUM)     ((NUM) < 16)

//CAN�ٶȶ���
/*
#define CAN_SPEED_33K     (uint8_t)(0)
#define CAN_SPEED_83K     (uint8_t)(1)
#define CAN_SPEED_50K     (uint8_t)(2)
#define CAN_SPEED_100K    (uint8_t)(3)
#define CAN_SPEED_125K    (uint8_t)(4)
#define CAN_SPEED_250K    (uint8_t)(5)
#define CAN_SPEED_500K    (uint8_t)(6)
#define CAN_SPEED_1000K   (uint8_t)(7)
*/
//���������
#define IS_CAN_SPEED(SPEED)    (((SPEED) == CAN_SPEED_33K)   || \
																((SPEED) == CAN_SPEED_83K)   || \
																((SPEED) == CAN_SPEED_50K)   || \
																((SPEED) == CAN_SPEED_100K)  || \
																((SPEED) == CAN_SPEED_125K)  || \
																((SPEED) == CAN_SPEED_250K)  || \
																((SPEED) == CAN_SPEED_500K)  || \
																((SPEED) == CAN_SPEED_1000K))						 

//Զ��֡��������֡
#define CAN_RTR_Data                ((uint8_t)0x00)  
#define CAN_RTR_Remote              ((uint8_t)0x01) 
#define IS_CAN_RTR(VALUE)   (((VALUE) == CAN_RTR_Data) || ((VALUE) == CAN_RTR_Remote))

//��׼֡������չ֡
#define CAN_IDE_Standard             ((uint8_t)0x00) 
#define CAN_IDE_Extended             ((uint8_t)0x01) 
#define IS_CAN_IDE(VALUE)  (((VALUE) == CAN_IDE_Standard) || (VALUE) == CAN_IDE_Extended)

//CAN�ж�Դ
#define CAN_IT_MB0     (uint16_t)(0)
#define CAN_IT_MB1     (uint16_t)(1)
#define CAN_IT_MB2     (uint16_t)(2)
#define CAN_IT_MB3     (uint16_t)(3)
#define CAN_IT_MB4     (uint16_t)(4)
#define CAN_IT_MB5     (uint16_t)(5)
#define CAN_IT_MB6     (uint16_t)(6)
#define CAN_IT_MB7     (uint16_t)(7)
#define CAN_IT_MB8     (uint16_t)(8)
#define CAN_IT_MB9     (uint16_t)(9)
#define CAN_IT_MB10    (uint16_t)(10)
#define CAN_IT_MB11    (uint16_t)(11)
#define CAN_IT_MB12    (uint16_t)(12)
#define CAN_IT_MB13    (uint16_t)(13)
#define CAN_IT_MB14    (uint16_t)(14)
#define CAN_IT_MB15    (uint16_t)(15)
#define IS_CAN_IT_MB(MB)  ((MB) < 16)


#define CAN_get_code(cs)			(((cs) & CAN_CS_CODE_MASK)>>24)
#define CAN_get_length(cs)    (((cs) & CAN_CS_DLC_MASK)>>16)

//������ʵ�ֵĽӿں���
void CAN_Init(CAN_InitTypeDef* CAN_InitStruct);
void CAN_EnableReceiveMB(CAN_Type* CANx,CAN_RxMsgTypeDef* RxMessage);
uint8_t CAN_Receive(CAN_Type* CANx,CAN_RxMsgTypeDef* RxMessage);
uint8_t CAN_Transmit(CAN_Type* CANx, CAN_TxMsgTypeDef* TxMessage);
void CAN_ITConfig(CAN_Type* CANx, uint16_t CAN_IT, FunctionalState NewState);
ITStatus CAN_GetITStatus(CAN_Type* CANx, uint16_t CAN_IT);
void CAN_ClearITPendingBit(CAN_Type* CANx, uint16_t CAN_IT);
void CAN_ClearAllITPendingBit(CAN_Type* CANx);










//Ĭ������
#define CAN_DEFAULT_TXID               (5)
#define CAN_DEFAULT_RXID               (6)
#define CAN_DEFAULT_TXMSGBOXINDEX      (1)
#define CAN_DEFAULT_RXMSGBOXINDEX      (2)
#define CAN_DEFAULT_SPEED              CAN_SPEED_125K

//CAN�����ٶ�ѡ��
typedef enum CAN_BAUDRATE_SELECT
{
	CAN_SPEED_33K,
	CAN_SPEED_83K,
	CAN_SPEED_50K,
	CAN_SPEED_100K,
	CAN_SPEED_125K,
	CAN_SPEED_250K,
	CAN_SPEED_500K,
	CAN_SPEED_1000K,
}CAN_BAUDRATE_SELECT;
//CAN ���ƽṹ
typedef struct can_dev
{
	uint16_t txid; //���͵�ַ
	uint16_t rxid; //���յ�ַ
	uint16_t txmsgboxindex; //����ʱ��ʹ�õ�����
	uint16_t rxmsgboxindex; //����ʱ��ʹ�õ�����
	CAN_BAUDRATE_SELECT baudrate; //�ٶ�ѡ��
	uint8_t  len; //����ÿ��֡���ݳ��� 0-8
	uint8_t reclen;//���յ�������֡����
}CAN_DevType_t;
extern CAN_DevType_t candev; //ʵ�廯can ���ƽṹ



#ifdef __cplusplus
}
#endif

#endif
