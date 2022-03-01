#ifndef _MK60_FLASH_H_
#define _MK60_FLASH_H_

//k66包含1024K的程序Flash  
//1024K的程序Flash分为256个扇区，每个扇区4K大小
//    sector（4K）为擦除最小单位
//    长字（32b）为写的最小单位

#include "common.h"

#define PROGRAM_CMD      			PGM8
#define SECTOR_SIZE     			(4096)
#define FLASH_SECTOR_NUM        	(256)                   //扇区数
#define FLASH_ALIGN_ADDR        	4                       //地址对齐整数倍
typedef uint32                  	FLASH_WRITE_TYPE;       //flash_write 函数写入 的数据类型




/*!
 *  @brief      使用宏定义对flash进行数据读取
 *  @param      sectorNo 		倒数扇区号（K66实际使用1~256）
 *  @param      offset	 		入扇区内部偏移地址（0~4095 中 4的倍数）
 *  @param      type		 	读取的数据类型
 *  @return     				返回给定地址的数据
 *  @notice     扇区不要使用前面的  程序存放在靠前的扇区，所以这里的sector_num是倒数的
 *  Sample usage:               FLASH_Read(1,0,uint32);//读取倒数第一个扇区偏移0数据类型为uint32
 */
#define     FLASH_Read(sectorNo,offset,type)        (*(type *)((uint32_t)(((FLASH_SECTOR_NUM - sectorNo)*SECTOR_SIZE) + (offset)))) 

/*!
 *  @brief      Flash 初始化   
 *  @return     无
 *  @notice     使用Flash时一定记得初始化
 *  Sample usage:  
 */
void FLASH_Init(void);

/*!
 *  @brief      获取单片机flash信息
 *  @return     扇区大小
 *  @notice     
 *  Sample usage:   FLASH_GetSectorSize();
 */
uint32 FLASH_GetSectorSize(void);


/*!
 *  @brief      擦出扇区
 *  @param      sectorNo 		倒数扇区号（K66实际使用1~256）
 *  @return     返回1擦除失败，返回0擦除成功
 *  @notice     
 *  Sample usage: uint32 dat = FLASH_GetSectorSize(10);
 */
uint8 FLASH_EraseSector(uint32 SectorNum);


/*!
 *  @brief      把数据写入扇区
 *  @param      sectorNo 		倒数扇区号（K66实际使用1~256）
 *  @param      offset	 		入扇区内部偏移地址（0~4095 中 4的倍数）
 *  @param      buf		 	    数据首地址
 *  @param      len		 	    数据长度
 *  @return     				返回给定地址的数据
 *  @notice     扇区不要使用前面的  程序存放在靠前的扇区，所以这里的sector_num是倒数的
 *  Sample usage:               FLASH_Read(1,0,uint32);//读取倒数第一个扇区偏移0数据类型为uint32
 */
uint8 FLASH_WriteBuf(uint32 SectorNum, const uint8 *buf, uint32 len, uint32 offset);



#endif //_FLASH_H_
