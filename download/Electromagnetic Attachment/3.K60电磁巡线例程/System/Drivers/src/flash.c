/**
  ******************************************************************************
  * @file    falsh.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.6.23
  * @brief   ����K60�̼��� Ƭ��flash �����ļ�
  ******************************************************************************
  */
#include "flash.h"
/***********************************************************************************************
 ���ܣ��ڲ����� ��������Ƿ����
 �βΣ�0
 ���أ�FLASH_OK �ɹ�    FLASH_ERROR ʧ�� 
 ��⣺
************************************************************************************************/
static uint8_t CommandLaunch(void)
{
    // ������ʴ����־λ�ͷǷ����ʱ�־λ 
    FTFL->FSTAT |=(FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK|FTFL_FSTAT_RDCOLERR_MASK);
    // ��������
    FTFL->FSTAT |= FTFL_FSTAT_CCIF_MASK;
    // �ȴ��������
    while((FTFL->FSTAT &FTFL_FSTAT_CCIF_MASK)==0);
    // �������־
    if(FTFL->FSTAT & (FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK|FTFL_FSTAT_MGSTAT0_MASK|FTFL_FSTAT_RDCOLERR_MASK))
    return(FLASH_ERROR); //����
    return (FLASH_OK); //�ɹ�
}

/***********************************************************************************************
 ���ܣ���ʼ��Ƭ��Flashģ��
 �βΣ�0
 ���أ�0
 ��⣺0
************************************************************************************************/
void FLASH_Init(void)
{
	//���FMC������
	FMC->PFB0CR |= FMC_PFB0CR_S_B_INV_MASK;
	FMC->PFB1CR |= FMC_PFB0CR_S_B_INV_MASK;
	// ��ֹ���Ź�
	WDOG->UNLOCK = 0xC520;
	WDOG->UNLOCK = 0xD928;
	WDOG->STCTRLH = 0;    // ��ֹ���Ź�
	//���Flash���ʴ���
  if(FTFL->FSTAT & FTFL_FSTAT_ACCERR_MASK)
  {
    FTFL->FSTAT |= FTFL_FSTAT_ACCERR_MASK;       //��������־
  }
  //��鱣������
  else if (FTFL->FSTAT & FTFL_FSTAT_FPVIOL_MASK)
  {
    FTFL->FSTAT |= FTFL_FSTAT_FPVIOL_MASK;
  }
  //������ͻ����
  else if (FTFL->FSTAT & FTFL_FSTAT_RDCOLERR_MASK)
  {
    FTFL->FSTAT |= FTFL_FSTAT_RDCOLERR_MASK;
  }
  //����Flashģ������ݻ���
  FMC->PFB0CR &= ~FMC_PFB0CR_B0DCE_MASK;
  FMC->PFB1CR &= ~FMC_PFB1CR_B1DCE_MASK;
}

/***********************************************************************************************
 ���ܣ�Ƭ��Flash��ȡ����
 �βΣ�FlashStartAdd : ��ַ
       len           : ��ȡ�ĳ���
       *pbuffer      : ������ָ��
 ���أ�0
 ��⣺
************************************************************************************************/
void FLASH_ReadByte(uint32_t FlashStartAdd,uint32_t len,uint8_t *pbuffer)
{
	uint32_t i = 0;
	for(i=0;i<len;i++)
	{
	  pbuffer[i] = *(uint8_t *)(FlashStartAdd+i);		//��ȡָ����ַ������
	}
}

/***********************************************************************************************
 ���ܣ�FALSHдһ������
 �βΣ�sectorNo      : ������(��ַ/2048)
       len           : ��ȡ�ĳ���
       *pbuffer      : ������ָ��
 ���أ�0
 ��⣺һ������2048�ֽ� ��������д2048�ֽ�
************************************************************************************************/
uint8_t FLASH_WriteSector(uint32_t sectorNo,uint16_t count,uint8_t const *buffer)
{
	uint16_t i;
	union
	{
		uint32_t  word;
		uint8_t   byte[4];
	} dest;
	dest.word = (uint32_t)(sectorNo*(1<<11));

	// ����д������
	FTFL->FCCOB0 = PGM4;
	// ���ֽڶ���
	for(i=0;i<count;i+=4)
	{
		// ���ô洢��ַ
		FTFL->FCCOB1 = dest.byte[2];
		FTFL->FCCOB2 = dest.byte[1];
		FTFL->FCCOB3 = dest.byte[0];
		// ��������	�����������а��մӵ�λ����λ�洢��
		FTFL->FCCOB4 = buffer[3];
		FTFL->FCCOB5 = buffer[2];
		FTFL->FCCOB6 = buffer[1];
		FTFL->FCCOB7 = buffer[0];
		dest.word+=4; buffer+=4;
		//��������Ƿ�ִ������ 
		if(FLASH_OK != CommandLaunch()) 
		return FLASH_ERROR;
    }
    return FLASH_OK;
}

/***********************************************************************************************
 ���ܣ�FALSH ����һ������
 �βΣ�sectorNo      : ������(��ַ/2048)
 ���أ�0
 ��⣺0
************************************************************************************************/
uint8_t FLASH_EraseSector(uint32_t sectorNo)
{
	union
	{
		uint32_t  word;
		uint8_t   byte[4];
	} dest;
	dest.word = (uint32_t)(sectorNo*(1<<11));
	// ���ò�������
	FTFL->FCCOB0 = ERSSCR; // ������������
	// ����Ŀ���ַ
	FTFL->FCCOB1 = dest.byte[2];
	FTFL->FCCOB2 = dest.byte[1];
	FTFL->FCCOB3 = dest.byte[0];
	//��������Ƿ�ִ������
		
	if(FLASH_OK == CommandLaunch())
	{
		return FLASH_OK;
	}
	else
	{
		return FLASH_ERROR;
	}
}

