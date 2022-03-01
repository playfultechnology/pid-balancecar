#ifndef _MK60_FLASH_H_
#define _MK60_FLASH_H_

//k66����1024K�ĳ���Flash  
//1024K�ĳ���Flash��Ϊ256��������ÿ������4K��С
//    sector��4K��Ϊ������С��λ
//    ���֣�32b��Ϊд����С��λ

#include "common.h"

#define PROGRAM_CMD      			PGM8
#define SECTOR_SIZE     			(4096)
#define FLASH_SECTOR_NUM        	(256)                   //������
#define FLASH_ALIGN_ADDR        	4                       //��ַ����������
typedef uint32                  	FLASH_WRITE_TYPE;       //flash_write ����д�� ����������




/*!
 *  @brief      ʹ�ú궨���flash�������ݶ�ȡ
 *  @param      sectorNo 		���������ţ�K66ʵ��ʹ��1~256��
 *  @param      offset	 		�������ڲ�ƫ�Ƶ�ַ��0~4095 �� 4�ı�����
 *  @param      type		 	��ȡ����������
 *  @return     				���ظ�����ַ������
 *  @notice     ������Ҫʹ��ǰ���  �������ڿ�ǰ�����������������sector_num�ǵ�����
 *  Sample usage:               FLASH_Read(1,0,uint32);//��ȡ������һ������ƫ��0��������Ϊuint32
 */
#define     FLASH_Read(sectorNo,offset,type)        (*(type *)((uint32_t)(((FLASH_SECTOR_NUM - sectorNo)*SECTOR_SIZE) + (offset)))) 

/*!
 *  @brief      Flash ��ʼ��   
 *  @return     ��
 *  @notice     ʹ��Flashʱһ���ǵó�ʼ��
 *  Sample usage:  
 */
void FLASH_Init(void);

/*!
 *  @brief      ��ȡ��Ƭ��flash��Ϣ
 *  @return     ������С
 *  @notice     
 *  Sample usage:   FLASH_GetSectorSize();
 */
uint32 FLASH_GetSectorSize(void);


/*!
 *  @brief      ��������
 *  @param      sectorNo 		���������ţ�K66ʵ��ʹ��1~256��
 *  @return     ����1����ʧ�ܣ�����0�����ɹ�
 *  @notice     
 *  Sample usage: uint32 dat = FLASH_GetSectorSize(10);
 */
uint8 FLASH_EraseSector(uint32 SectorNum);


/*!
 *  @brief      ������д������
 *  @param      sectorNo 		���������ţ�K66ʵ��ʹ��1~256��
 *  @param      offset	 		�������ڲ�ƫ�Ƶ�ַ��0~4095 �� 4�ı�����
 *  @param      buf		 	    �����׵�ַ
 *  @param      len		 	    ���ݳ���
 *  @return     				���ظ�����ַ������
 *  @notice     ������Ҫʹ��ǰ���  �������ڿ�ǰ�����������������sector_num�ǵ�����
 *  Sample usage:               FLASH_Read(1,0,uint32);//��ȡ������һ������ƫ��0��������Ϊuint32
 */
uint8 FLASH_WriteBuf(uint32 SectorNum, const uint8 *buf, uint32 len, uint32 offset);



#endif //_FLASH_H_
