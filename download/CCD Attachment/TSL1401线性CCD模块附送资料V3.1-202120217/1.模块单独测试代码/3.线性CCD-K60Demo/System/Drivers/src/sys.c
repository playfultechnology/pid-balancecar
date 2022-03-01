/**
  ******************************************************************************
  * @file    sys.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� ϵͳ��API����
  ******************************************************************************
  */
#include "sys.h"
CPUInfoType_t CPUInfo; //������������Ϣ�ṹ��
extern uint32_t SystemCoreClock;
/***********************************************************************************************
 ���ܣ����ϵͳ��Ϣ
 �βΣ�0
 ���أ�0
 ��⣺��ô������ͺ� ����   ����ں�ʱ��,����ʱ�ӣ�FlexBusʱ�ӣ�Flashʱ�� ��     
************************************************************************************************/
void GetCPUInfo(void)
{
	//���Ȼ�ø�λ״̬��Ϣ
	if (MC->SRSH & MC_SRSH_SW_MASK)     CPUInfo.ResetState = 1;
	if (MC->SRSH & MC_SRSH_LOCKUP_MASK) CPUInfo.ResetState = 2;
	if (MC->SRSH & MC_SRSH_JTAG_MASK)   CPUInfo.ResetState = 3;
	if (MC->SRSL & MC_SRSL_POR_MASK)    CPUInfo.ResetState = 4;
	if (MC->SRSL & MC_SRSL_PIN_MASK)  	CPUInfo.ResetState = 5;
	if (MC->SRSL & MC_SRSL_COP_MASK)    CPUInfo.ResetState = 6;
	if (MC->SRSL & MC_SRSL_LOC_MASK)    CPUInfo.ResetState = 7;
	if (MC->SRSL & MC_SRSL_LVD_MASK)    CPUInfo.ResetState = 8;
	if (MC->SRSL & MC_SRSL_WAKEUP_MASK) CPUInfo.ResetState = 9;
	//ȷ��Kinetisϵ���ͺ�
	switch((SIM->SDID & SIM_SDID_FAMID(0x7))>>SIM_SDID_FAMID_SHIFT) 
	{  
    	case 0x0: CPUInfo.FamilyType = 10; break;
    	case 0x1: CPUInfo.FamilyType = 20; break;
		  case 0x2: CPUInfo.FamilyType = 30; break;
			case 0x3: CPUInfo.FamilyType = 40; break;
			case 0x4: CPUInfo.FamilyType = 60; break;
			case 0x5: CPUInfo.FamilyType = 70; break;
			case 0x6: CPUInfo.FamilyType = 50; break;
			case 0x7: CPUInfo.FamilyType = 53; break;
			 default: CPUInfo.FamilyType = 0;  break;
	}
	//��װ��Ϣ
	switch((SIM->SDID & SIM_SDID_PINID(0xF))>>SIM_SDID_PINID_SHIFT) 
	{  
		case 0x02: CPUInfo.PinCnt = 32;  break;
		case 0x04: CPUInfo.PinCnt = 48;  break;
		case 0x05: CPUInfo.PinCnt = 64;  break;
		case 0x06: CPUInfo.PinCnt = 80;  break;
		case 0x07: CPUInfo.PinCnt = 81;  break;
		case 0x08: CPUInfo.PinCnt = 100; break;
		case 0x09: CPUInfo.PinCnt = 104; break;
		case 0x0A: CPUInfo.PinCnt = 144; break;
		case 0x0C: CPUInfo.PinCnt = 196; break;
 		case 0x0E: CPUInfo.PinCnt = 256; break;
	  default:  CPUInfo.PinCnt = 0;   break;	
	}  
   //SiliconRevID
    switch((SIM->SDID & SIM_SDID_REVID(0xF))>>SIM_SDID_REVID_SHIFT) 
    { 
			case 0x0: CPUInfo.SiliconRev = 10;break;
			case 0x1: CPUInfo.SiliconRev = 11;break;
			case 0x2: CPUInfo.SiliconRev = 12;break;
			default:  CPUInfo.SiliconRev = 0; break;
    }	
	//PFlash��С
	switch((SIM->FCFG1 & SIM_FCFG1_PFSIZE(0xF))>>SIM_FCFG1_PFSIZE_SHIFT)
	{
		case 0x7: CPUInfo.PFlashSize = 128*1024;break;
		case 0x9: CPUInfo.PFlashSize = 256*1024;break;
		case 0xB: CPUInfo.PFlashSize = 512*1024;break;
		case 0xF: CPUInfo.PFlashSize = 512*1024;break;
	 	default:  CPUInfo.PFlashSize = 0*1024;  break; 		
	}
	//�����FlexNVM ȷ��FlexNVM��С
	if (SIM->FCFG2 & SIM_FCFG2_PFLSH_MASK) 
	{
		CPUInfo.FlexNVMSize = 0;
	}
	else
	{
		//ȷ��FLexNVM��С
		switch((SIM->FCFG1 & SIM_FCFG1_NVMSIZE(0xF))>>SIM_FCFG1_NVMSIZE_SHIFT)
		{
			case 0x0: CPUInfo.FlexNVMSize = 0;break;
			case 0x7: CPUInfo.FlexNVMSize = 128*1024;break;
			case 0x9: CPUInfo.FlexNVMSize = 256*1024;break;
			case 0xF: CPUInfo.FlexNVMSize = 256*1024;break;
			default:  CPUInfo.FlexNVMSize = 0;break; 		
		}
	}
	//RAM��С
	switch((SIM->SOPT1 & SIM_SOPT1_RAMSIZE(0xF))>>SIM_SOPT1_RAMSIZE_SHIFT)
	{
		case 0x5: CPUInfo.RAMSize = 32*1024;  break;
		case 0x7: CPUInfo.RAMSize = 64*1024;  break;
		case 0x8: CPUInfo.RAMSize = 96*1024;  break;
		case 0x9: CPUInfo.RAMSize = 128*1024; break;
		default:  CPUInfo.RAMSize = 0*1024;   break;	
	}
	//ʱ�Ӽ���
	//SystemCoreClockUpdate();            //���¼���CoreClock
	CPUInfo.CoreClock = SystemCoreClock;
	CPUInfo.BusClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV2_MASK)>>SIM_CLKDIV1_OUTDIV2_SHIFT)+1));     //����BusClock
	CPUInfo.FlexBusClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV3_MASK)>>SIM_CLKDIV1_OUTDIV3_SHIFT)+1)); //����FlexBusClock
	CPUInfo.FlashClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV4_MASK)>>SIM_CLKDIV1_OUTDIV4_SHIFT)+1));   //����FlashClock
}
/***********************************************************************************************
 ���ܣ����� ����ϵͳ��Ƶ
 �βΣ�ClockOption ʱ��Դѡ��
			 @arg  ClockSource_IRC    : �ڲ�ʱ��Դ
			 @arg  ClockSource_EX8M   : �ⲿ8M����
			 @arg  ClockSource_EX50M  : �ⲿ50M����
			 CoreClock   ϣ���ﵽ���ں�Ƶ��
			 @arg  CoreClock_48M : CoreClock = 48M
			 @arg  CoreClock_64M : CoreClock = 64M
			 @arg  CoreClock_72M : CoreClock = 72M
			 @arg  CoreClock_96M : CoreClock = 96M
			 @arg  CoreClock_100M : CoreClock = 100M
 ���أ�0
 ��⣺�������е�Ƶ�ʶ��ܴﵽ ���øú��� �Ƽ�ʹ��DisplayCPUInfo����ʾһ����Ƶ
       SystemClockSetup(ClockSource_EX50M,CoreClock_100M) �ⲿ50M���� PLL��Ƶ��CoreClock = 100M
************************************************************************************************/
void SystemClockSetup(uint8_t ClockOption,uint16_t CoreClock)
{
	//������
	assert_param(IS_CLOCK_OPTION(ClockOption));
	assert_param(IS_CLOCK_SELECT(CoreClock));
	//ʹ���ڲ�����
		//��Ƶ����:  
		//SIM_CLKDIV1_OUTDIV1(0) CORE     CLOCK  1��Ƶ   UP TO 100M  
		//SIM_CLKDIV1_OUTDIV2(1) BUS      CLOCK  2��Ƶ   UP TO 50M 
		//SIM_CLKDIV1_OUTDIV3(1) FlexBus  ClOCK  2��Ƶ   UP TO 50M 
		//SIM_CLKDIV1_OUTDIV4(3) Flash    ClOCK  3��Ƶ   UP TO 25M 
	if(ClockOption == ClockSource_IRC)
	{
		SIM->CLKDIV1 = (uint32_t)0xFFFFFFFFu; //����ϵͳԤ��Ƶ�� ������Ϊ��Ϊ��ͷ�Ƶ
		// ת�� FEI ģʽ 
		MCG->C1 = (uint8_t)0x06u;
		MCG->C2 = (uint8_t)0x00u;
		MCG->C4 &= ~((1<<6)|(1<<7)|(1<<5));
		switch(CoreClock)
		{
			case CoreClock_96M:
				MCG->C4|= (1<<6)|(1<<7)|(1<<5);  
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));
				SystemCoreClock	= 96000000;
				break;
			case CoreClock_72M:
				MCG->C4|= (1<<6)|(1<<7)|(0<<5);  
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(2));	
				SystemCoreClock	= 72000000;
				break;
			case CoreClock_48M:
				MCG->C4|= (0<<6)|(1<<7)|(1<<5);  
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(0)|SIM_CLKDIV1_OUTDIV3(0)|SIM_CLKDIV1_OUTDIV4(1));	
				SystemCoreClock	= 48000000;
				break;
			default: //48M
				MCG->C4|= (0<<6)|(1<<7)|(1<<5);  
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(0)|SIM_CLKDIV1_OUTDIV3(0)|SIM_CLKDIV1_OUTDIV4(1));	
				SystemCoreClock	= 48000000;
				break;
		}
		MCG->C5 = (uint8_t)0x00u;
		MCG->C6 = (uint8_t)0x00u;
		while((MCG->S & MCG_S_IREFST_MASK) == 0u);  //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
		while((MCG->S & 0x0Cu) != 0x00u);           //�ȴ�FLL��ѡ��
	}
	if(ClockOption == ClockSource_EX8M || ClockOption == ClockSource_EX50M)
	{
		SIM->CLKDIV1 = (uint32_t)0xFFFFFFFFu; //����ϵͳԤ��Ƶ�� ������Ϊ��Ϊ��ͷ�Ƶ
		//ת�� FBE ģʽ 
		OSC->CR = (uint8_t)0x00u;
		SIM->SOPT2 &= (uint8_t)~(uint8_t)0x01u;
		MCG->C2 = (uint8_t)0x24u;
		MCG->C1 = (uint8_t)0x9Au;
		MCG->C4 &= (uint8_t)~(uint8_t)0xE0u;
		MCG->C5 = (uint8_t)0x03u;
		MCG->C6 = (uint8_t)0x00u;
		while((MCG->S & MCG_S_OSCINIT_MASK) == 0u);//��� FLL�ο�ʱ�����ڲ��ο�ʱ�� 
		while((MCG->S & MCG_S_IREFST_MASK) != 0u); //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
		while((MCG->S & 0x0Cu) != 0x08u);          //�ȴ� FBE ��ѡ��
		if(ClockOption == ClockSource_EX8M)
		{
			MCG->C5 = (uint8_t)MCG_C5_PRDIV(3);        //8/4 = 2M
		}
		else if (ClockOption == ClockSource_EX50M)
		{
			MCG->C5 = (uint8_t)MCG_C5_PRDIV(24);        //50/25 = 2M
		}
		switch(CoreClock)
		{
			case CoreClock_48M:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(0));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(0)|SIM_CLKDIV1_OUTDIV3(0)|SIM_CLKDIV1_OUTDIV4(1));	
				SystemCoreClock	= 48000000;
				break;		
			case CoreClock_64M:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(8));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(2));
				SystemCoreClock	= 64000000;
				break;		
			case CoreClock_72M:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(12));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(2));
				SystemCoreClock	= 72000000;
				break;			
			case CoreClock_96M:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(24));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));		
				SystemCoreClock	= 96000000;
				break;		
			case CoreClock_100M:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(26));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));	
				SystemCoreClock	= 100000000;
				break;	
			case CoreClock_200M:
				if(ClockOption == ClockSource_EX8M)
				{
					MCG->C5 = (uint8_t)MCG_C5_PRDIV(1);        //8/2 = 4M
					MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(26));
					SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(7));	
				}
				else if (ClockOption == ClockSource_EX50M)
				{
					MCG->C5 = (uint8_t)MCG_C5_PRDIV(12);
					MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(28));
					SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(7));	
				}
				SystemCoreClock	= 200000000;
				break;			
			default:
				MCG->C6 = (uint8_t)(0x40u|MCG_C6_VDIV(0));
				SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(0)|SIM_CLKDIV1_OUTDIV3(0)|SIM_CLKDIV1_OUTDIV4(1));
				SystemCoreClock	= 48000000;
				break;
		} 
		while((MCG->S & MCG_S_PLLST_MASK) == 0u);   //�ȴ�PLLS ʱ��Դת�� PLL
		while((MCG->S & MCG_S_LOCK_MASK) == 0u);    //�ȴ�����
		//ת��PEE
		MCG->C1 = (uint8_t)0x1Au;
		while((MCG->S & 0x0Cu) != 0x0Cu);           //�ȴ�PLL���
		while((MCG->S & MCG_S_LOCK_MASK) == 0u);      //�ȴ�PLL����
	}
	GetCPUInfo();//��ȡCPU��Ϣ ���¼����������Ƶ��
}
/***********************************************************************************************
 ���ܣ�����һ���жϵ����ȼ�
 �βΣ�IRQn �жϺ�
			 PriorityGroup ���ȼ����� 
		   @arg NVIC_PriorityGroup_0
			 @arg NVIC_PriorityGroup_1
		   @arg NVIC_PriorityGroup_2
			 @arg NVIC_PriorityGroup_3
		   @arg NVIC_PriorityGroup_4
		 	 PreemptPriority  ��ռ���ȼ�
			 SubPriority      �����ȼ�(��Ӧ���ȼ�)
 ���أ�0
 ��⣺��������жϵ���ռ���ȼ�����Ӧ���ȼ���һ���Ļ� �������ж��ȷ�������ִ��
			 �����ȼ�����ռ���ȼ��ǿ��Դ�ϵ����ȼ�����ռ���ȼ��жϵ�
			 ����ռ���ȼ���ͬ���ж� �����ȼ�����Ӧ���ȼ������Դ�ϵ����ȼ�����Ӧ���ȼ� 
************************************************************************************************/
void NVIC_Init(IRQn_Type IRQn,uint32_t PriorityGroup,uint32_t PreemptPriority,uint32_t SubPriority)
{
	//������
  assert_param(IS_NVIC_PRIORITY_GROUP(PriorityGroup));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));  
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
	//����NVIC�жϷ���
	NVIC_SetPriorityGrouping(PriorityGroup);
	//�����ж����ȼ�
	NVIC_SetPriority(IRQn,NVIC_EncodePriority(PriorityGroup,PreemptPriority,SubPriority));
}
/***********************************************************************************************
 ���ܣ���λ
 �βΣ�0
 ���أ�0
 ��⣺��λ
************************************************************************************************/
void SystemSoftReset(void)
{   
	//SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	 
	NVIC_SystemReset();
} 
/***********************************************************************************************
 ���ܣ�ʹ�����ж�
 �βΣ�0
 ���أ�0
 ��⣺����  CMSIS Cortex-M Core Function Access �� ���� ��.core_cmFunc.h
************************************************************************************************/
void EnableInterrupts(void)
{
	__enable_irq();
}
/***********************************************************************************************
 ���ܣ��ر����ж�
 �βΣ�0
 ���أ�0
 ��⣺����  CMSIS Cortex-M Core Function Access �� ���� ��.core_cmFunc.h
************************************************************************************************/
void DisableInterrupts(void)
{
	__disable_irq();
}
/***********************************************************************************************
 ���ܣ������ж���������ʼλ��
 �βΣ�0
 ���أ�0
 ��⣺
************************************************************************************************/
void SetVectorTable(uint32_t VectorOffset)
{
	//������
  assert_param(IS_VECTOR_OFFSET(VectorOffset));
	SCB->VTOR = VectorOffset;  //�����ж�������ƫ�Ƶ�ַ
}
/***********************************************************************************************
 ���ܣ���ù̼���汾
 �βΣ�0
 ���أ��̼���汾
 ��⣺
************************************************************************************************/
uint16_t GetFWVersion(void)
{
	return(FW_VERSION);
}
