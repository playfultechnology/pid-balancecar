#ifndef __USB_H_
#define	__USB_H_
#include "sys.h"
#include "message_manage.h"
#include "stdio.h"
/***********************************************************************************************
//CH_Kinetis������  V2.3   
//����    :YANDLD 
//E-MAIL  :yandld@126.com
//�޸�����:2013/2/14
//�汾��V2.3
//�Ա���http://upcmcu.taobao.com
//QQ    1453363089
//Copyright(C) YANDLD 2012-2022
//All rights reserved
************************************************************************************************/
/*ѡ��ǰUSB�豸������*/
#define USB_DEVICE_CLASS USB_DEVICE_CLASS_HID

//���ò����궨��
#define BIT_SET(BitNumber, Register)        (Register |=(1<<BitNumber))
#define BIT_CLR(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define BIT_CHK(BitNumber, Register)        (Register & (1<<BitNumber))

//������ʹ�õ��������ڵ�
//���ʹ�ö˵�3
#define EP_OUT          (3)
//����ʹ�ö˵�2
#define EP_IN           (2)

//USB��Ƶ����
#define USB_FARCTIONAL_VALUE    0x02

//EP0����������
#define EP0_SIZE            32

//EP1����
#define EP1_VALUE           _EP_IN
#define EP1_TYPE            INTERRUPT
#define EP1_SIZE            32
#define EP1_BUFF_OFFSET     0x18

//EP2����
#define EP2_VALUE           _EP_IN
#define EP2_TYPE            BULK
#define EP2_SIZE            32
#define EP2_BUFF_OFFSET     0x20

//EP3����
#define EP3_VALUE           _EP_OUT
#define EP3_TYPE            BULK
#define EP3_SIZE            32
#define EP3_BUFF_OFFSET     0x28

//EP4����
#define EP4_VALUE           DISABLE
#define EP4_SIZE            1
#define EP4_BUFF_OFFSET     0x08

//EP5����
#define EP5_VALUE           DISABLE
#define EP5_SIZE            1
#define EP5_BUFF_OFFSET     0x08

//EP6����
#define EP6_VALUE           DISABLE
#define EP6_SIZE            1
#define EP6_BUFF_OFFSET     0x08


//����ʹ�õ����� ��� �˵�
#define _EP_IN      USB_ENDPT_EPTXEN_MASK //ʱ�ܸö˵�����봫�� //USB���е�������������������˵
#define _EP_OUT     USB_ENDPT_EPRXEN_MASK //ʱ�ܸö˵��������� //USB���е�������������������˵

#define DISABLE 0


// BDT״̬
//DBT��MCU����
#define kMCU      0x00
//DBT��SIE(USB)ģ�����
#define kSIE      0x80

//BDT������DATA0��ʽ
#define kUDATA0   0x88
//BDT������DATA1��ʽ
#define kUDATA1   0xC8

//USB�����У�����PID����
#define mSETUP_TOKEN    0x0D
#define mOUT_TOKEN      0x01
#define mIN_TOKEN       0x09


/***********************************************************************************************
//��׼��SETUP��������  �ڱ�׼�����8���ֽ��еĵڶ���
************************************************************************************************/
#define mGET_STATUS           0
#define mCLR_FEATURE          1
#define mSET_FEATURE          3
#define mSET_ADDRESS          5
#define mGET_DESC             6
#define mSET_DESC             7
#define mGET_CONFIG           8
#define mSET_CONFIG           9
#define mGET_INTF             10
#define mSET_INTF             11
#define mSYNC_FRAME           12
#define	mGET_MAXLUN	          0xFE

//��ȡ������
#define DEVICE_DESCRIPTOR         1
#define CONFIGURATION_DESCRIPTOR  2
#define STRING_DESCRIPTOR         3
#define INTERFACE_DESCRIPTOR      4
#define ENDPOINT_DESCRIPTOR       5
#define REPORT_DESCRIPTOR         0x22

/*����USB2.0��׼*/
/*����USB�豸������*/
#define USB_DEVICE_CLASS_AUDIO        1
#define USB_DEVICE_CLASS_CDC          2
#define USB_DEVICE_CLASS_HID          3
#define USB_DEVICE_CLASS_PHY          4
#define USB_DEVICE_CLASS_IMAGE        5
#define USB_DEVICE_CLASS_MASS_STORAGE 6
#define USB_DEVICE_CLASS_HUB          7
#define USB_DEVICE_CLASS_CDC_DATA     8
#define USB_DEVICE_CLASS_SMARTCARD    9
//.......
/***********************************************************************************************
// SETUP�������� ��USB��׼����ṹ�� bmRequestType ��
************************************************************************************************/
enum
{
    uSETUP,
    uDATA
};

enum
{
    EP0,
    EP1,
    EP2,
    EP3,
    EP4,
    EP5,
    DUMMY,
    LOADER
    
};

enum
{
    uPOWER,
    uENUMERATED,
    uENABLED,
    uADDRESS,
    uREADY    
};
enum
{
    fIN,
    fOUT
};

enum
{
    bEP0OUT_ODD,
    bEP0OUT_EVEN,
    bEP0IN_ODD,
    bEP0IN_EVEN,
    bEP1OUT_ODD,
    bEP1OUT_EVEN,
    bEP1IN_ODD,
    bEP1IN_EVEN,
    bEP2OUT_ODD,
    bEP2OUT_EVEN,
    bEP2IN_ODD,
    bEP2IN_EVEN,
    bEP3OUT_ODD,
    bEP3OUT_EVEN,
    bEP3IN_ODD,
    bEP3IN_EVEN
};
/***********************************************************************************************
 ��������������(BDT)�ṹ��
 ÿ���˵�2��BDT(һ������΢��������һ������USBģ��) ÿ�� BDT 8�ֽ�
************************************************************************************************/
typedef union _tBDT_STAT
{
    uint8_t _byte;
	//���͵�MCU�����ֶ�
    struct{
        uint8_t :1;
        uint8_t :1;
        uint8_t BSTALL:1;              //OTGģ�鷢��һ������Э��
        uint8_t DTS:1;                 //
        uint8_t NINC:1;                //�������ݻ�����ʱ��DMA���治���������ĵ�ַ
        uint8_t KEEP:1;                //
        uint8_t DATA:1;                //���ͻ������DATA0/DATA1����USBģ�鲻�ı��λ
        uint8_t UOWN:1;                //BDT����Ȩ UOWN=1 USBģ��ӵ�У�UOWN=0 ΢������ӵ��
    }McuCtlBit;
    //���ܿ����ֶ� 
    struct{
        uint8_t    :2;
        uint8_t PID:4;                 //����־
        uint8_t    :2;
    }RecPid;
} tBDT_STAT,*ptBDT_STAT;                            //��������������ṹ��
//BDT����������������
typedef struct _tBDT
{
    tBDT_STAT Stat;
    uint8_t  dummy;
    uint16_t Cnt;     //���ܵ����ֽ���
    uint32_t Addr;    //��������ַ         
  } tBDT,*ptBDT;

/***********************************************************************************************
 SETUP���ṹ��
************************************************************************************************/
typedef struct _tUSB_Setup 
{
       uint8_t bmRequestType; //D7:���䷽�� D[6:5]���� D[4:0]���ն� 
       uint8_t bRequest;      //�ض�����
       uint8_t wValue_l;      //�ִ�С�ֶ�,��������Ĳ�ͬ����ͬ
       uint8_t wValue_h;      
       uint8_t wIndex_l;      //�ִ�С�ֶ�,��������Ĳ�ͬ����ͬ,ͨ���Ǵ���������λ����
       uint8_t wIndex_h;
       uint8_t wLength_l;     //��������ݽ׶Σ������ʾ��Ҫ������ֽڴ�С
       uint8_t wLength_h;
}tUSB_Setup;

//BDT ����������
extern tBDT tBDTtable[16];													//�ڲ�SRAM�ڴ��

//������ʵ�ֵĽӿں���
void USB_WaitDeviceEnumed(void);
uint8_t USB_IsDeviceEnumed(void);
void USB_EP_IN_Transfer(uint8_t uint8_tEP,uint8_t *puint8_tDataPointer,uint8_t uint8_tDataSize);
uint16_t USB_EP_OUT_SizeCheck(uint8_t uint8_tEP);
void USB_EnableInterface(void);
void USB_GetDescHandler(void);
uint8_t USB_Init(void);
void USB_DisConnect(void);
void USB_Connect(void);

#endif

