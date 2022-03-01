#include "include.h"
#include "MK60_FLASH.h"

// flash commands 
#define RD1BLK    0x00  // read 1 block
#define RD1SEC    0x01  // read 1 section
#define PGMCHK    0x02  // program check 
#define RDRSRC    0x03  // read resource 
#define PGM4      0x06  // program phase program 4 byte 
#define PGM8      0x07  // program phase program 8 byte 
#define ERSBLK    0x08  // erase flash block 
#define ERSSCR    0x09  // erase flash sector 
#define PGMSEC    0x0B  // program section 
#define RD1ALL    0x40  // read 1s all block 
#define RDONCE    0x41  // read once 
#define PGMONCE   0x43  // program once 
#define ERSALL    0x44  // erase all blocks 
#define VFYKEY    0x45  // verift backdoor key 
#define PGMPART   0x80  // program paritition 
#define SETRAM    0x81  // set flexram function 
#define NORMAL_LEVEL 0x0


// disable interrupt before lunch command 
#define CCIF    (1<<7)
#define ACCERR  (1<<5)
#define FPVIOL  (1<<4)
#define MGSTAT0 (1<<0)






volatile uint8 s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8 *reg);
flash_run_entry_t s_flash_run_entry;
    

//�ڲ�ʹ��
__STATIC_INLINE uint8 FlashCmdStart(void)
{
	//�����������־λ
    FTFL->FSTAT = ACCERR | FPVIOL;
    s_flash_run_entry = (flash_run_entry_t)((uint32)s_flash_command_run + 1);
    s_flash_run_entry(&FTFL->FSTAT);
    
    if(FTFL->FSTAT & (ACCERR | FPVIOL | MGSTAT0)) return 1;	//���ִ���
    return 0;//�ɹ�

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��ʼ��
//  @return     				����һ�������Ĵ�С
//  @since      v1.0
//  Sample usage:               uint32 dat = FLASH_GetSectorSize();
//-------------------------------------------------------------------------------------------------------------------
uint32 FLASH_GetSectorSize(void)
{
    return SECTOR_SIZE;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��ʼ��
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void FLASH_Init(void)
{
    //���״̬��ʶ
    FTFL->FSTAT = ACCERR | FPVIOL;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��������
//  @param      SectorNum 		��Ҫ�������������
//  @return     				����1����ʧ�ܣ�����0�����ɹ�
//  @since      v1.0
//  Sample usage:               uint32 dat = FLASH_GetSectorSize(10);
//-------------------------------------------------------------------------------------------------------------------
uint8 FLASH_EraseSector(uint32 SectorNum)
{
    int ret;
	
	union
	{
		uint32  word;
		uint8   byte[4];
	} dest;

	dest.word = (uint32)(FLASH_SECTOR_NUM - SectorNum)*SECTOR_SIZE;

	//��������
	FTFL->FCCOB0 = ERSSCR; 
	FTFL->FCCOB1 = dest.byte[2];
	FTFL->FCCOB2 = dest.byte[1];
	FTFL->FCCOB3 = dest.byte[0];
    DisableInterrupts;
    ret = FlashCmdStart();
    EnableInterrupts;
    
    return ret;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH����д�뵽����
//  @param      SectorNum 		��Ҫд����������
//  @param      *buf	 		�����׵�ַ
//  @param      len		 		д����ֽ��� �ر�����һ��uint16�����������ֽ� һ��uint32�������ĸ��ֽ�
//  @param      offset		 	����Ϊ4��������
//  @return     				����1д��ʧ�ܣ�����0д��ɹ�
//  @since      v1.0
//  Sample usage:               FLASH_WriteSector(10,(const uint8 *)buf,4,0);//��buf���������ǰ��λ����д�뵽10������(ÿһλ��uint8����)
//-------------------------------------------------------------------------------------------------------------------
uint8 FLASH_WriteBuf(uint32 SectorNum, const uint8 *buf, uint32 len, uint32 offset)
{
    uint16 step, ret, i;
	union
	{
		uint32  word;
		uint8   byte[4];
	} dest;
	dest.word = (uint32)(FLASH_SECTOR_NUM - SectorNum)*SECTOR_SIZE + offset;

	FTFL->FCCOB0 = PROGRAM_CMD;

    step = 4;

	for(i=0; i<len; i+=step)
	{
        //���õ�ַ
		FTFL->FCCOB1 = dest.byte[2];
		FTFL->FCCOB2 = dest.byte[1];
		FTFL->FCCOB3 = dest.byte[0];
		//��������
		FTFL->FCCOB4 = buf[3];
		FTFL->FCCOB5 = buf[2];
		FTFL->FCCOB6 = buf[1];
		FTFL->FCCOB7 = buf[0];
        
		dest.word += step; buf += step;

        DisableInterrupts;
        ret = FlashCmdStart();
        EnableInterrupts;
        
		if(ret) return ret;
    }
    return ret;
}
