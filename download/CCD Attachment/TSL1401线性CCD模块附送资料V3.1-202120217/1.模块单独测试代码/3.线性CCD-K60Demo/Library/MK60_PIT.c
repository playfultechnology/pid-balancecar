
#include "include.h"
#include "MK60_PIT.h"

/*------------------------------------------------------------------------------------------------------
����    ����PIT_Init
����    �ܡ�PIT��ʱ����ʱ�жϳ�ʼ��
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
����    ����ms �ж�ʱ�䣬��λms
���� �� ֵ����
��ʵ    ����pit_init(PIT0,1000); // PIT0�жϣ�1000ms����1s����PIT0_interrupt()һ��
��ע�����ע����Ҫʹ��NVIC_SetPriority������PIT�ж����ȼ�   NVIC_EnableIRQ��ʹ���ж�
��ע�����
��ע��������ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
��ע�����NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
��ע�����NVIC_EnableIRQ(PIT0_IRQn);			          //ʹ��PIT0_IRQn���ж�
--------------------------------------------------------------------------------------------------------*/
void PIT_Init(PITn pitn, uint32_t ms)
{
    //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

    /* ��������ж�ʱ�� */
    PIT_LDVAL(pitn)  = ms * bus_clk * 1000;

    /* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);

    /* ʹ�� PITn��ʱ��,����PITn�ж� */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );

}


/*------------------------------------------------------------------------------------------------------
����    ����PIT_delayms
����    �ܡ�PIT��ʱ����ʱ��ʱ
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
����    ����ms  ��ʱʱ�䣬��λms
���� �� ֵ����
��ʵ    ����PIT_delayms(PIT0,1000); // PIT0��ʱ��1000ms
��ע�����
--------------------------------------------------------------------------------------------------------*/
void PIT_Delayms(PITn pitn, uint32_t ms)
{
    //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* ��������ж�ʱ�� */
    PIT_LDVAL(pitn)  = ms * bus_clk * 1000;

	/* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);

    /* ��ռ����� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* ��ֹPITn�ж� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* ʹ�� PITn��ʱ�� */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;

    /* �ȴ�ʱ�䵽 */
	while( !(PIT_TFLG(pitn) & PIT_TFLG_TIF_MASK) ){}

    /* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);
}


/*------------------------------------------------------------------------------------------------------
����    ����PIT_delayus
����    �ܡ�PIT��ʱ����ʱ��ʱ
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
����    ����us  ��ʱʱ�䣬��λus
���� �� ֵ����
��ʵ    ����PIT_delayms(PIT0,1000); // PIT0��ʱ��1000us
��ע�����
--------------------------------------------------------------------------------------------------------*/
void PIT_Delayus(PITn pitn, uint32_t us)
{
    //PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* ��������ж�ʱ�� */
    PIT_LDVAL(pitn)  = us * bus_clk;

	/* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);

    /* ��ռ����� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* ��ֹPITn�ж� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* ʹ�� PITn��ʱ�� */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;

    /* �ȴ�ʱ�䵽 */
	while( !(PIT_TFLG(pitn) & PIT_TFLG_TIF_MASK) ){}

    /* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);
}


/*------------------------------------------------------------------------------------------------------
����    ����PIT_TimeStart
����    �ܡ�PIT��ʱ����ʱ��ʱ��ʼ
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
���� �� ֵ����
��ʵ    ����PIT_TimeStart(PIT0); // PIT0��ʱ ���������ʱ����
��ע�����
--------------------------------------------------------------------------------------------------------*/
void PIT_TimeStart(PITn pitn)
{
	//PIT �õ��� Bus Clock ����Ƶ��

    /* ����ʱ��*/
	SIM->SCGC6       |= SIM_SCGC6_PIT_MASK;

    /* ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼������� */
	PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );

	/* ��������ж�ʱ�� */
    PIT_LDVAL(pitn)  = 0xFFFFFFFF;

	/* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);

    /* ��ռ����� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

    /* ��ֹPITn�ж� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;

	/* ʹ�� PITn��ʱ�� */
    PIT_TCTRL(pitn) |= PIT_TCTRL_TEN_MASK;
}

/*------------------------------------------------------------------------------------------------------
����    ����PIT_TimeGet
����    �ܡ�PIT��ʱ���õ���ʱʱ��
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
���� �� ֵ����ʱʱ�� ��λus
��ʵ    ����PIT_TimeGet(PIT0);  // PIT0��ʱ ���������ʱ����
��ע�����ע��Ҫ����PIT_TimeStart(PIT0);������ʱ��
��ע�����ע��PITΪ32Ϊ��ʱ��  ����ʱ (0xFFFFFFFF / Bus Clock ����Ƶ��) s
--------------------------------------------------------------------------------------------------------*/
uint32_t PIT_TimeGet(PITn pitn)
{
	return (0xFFFFFFFF - PIT_CVAL(pitn)) / bus_clk;
}

/*------------------------------------------------------------------------------------------------------
����    ����PIT_TimeGet
����    �ܡ�PIT��ʱ���õ���ʱʱ��
����    ����pitn:ģ����PIT0��PIT1��PIT2��PIT3
���� �� ֵ����ʱʱ�� ��λus
��ʵ    ����PIT_TimeGet(PIT0);  // PIT0��ʱ ���������ʱ����
��ע�����ע��Ҫ����PIT_TimeStart(PIT0);������ʱ��
��ע�����ע��PITΪ32Ϊ��ʱ��  ����ʱ (0xFFFFFFFF / Bus Clock ����Ƶ��) s
--------------------------------------------------------------------------------------------------------*/
void PIT_Close(PITn pitn)
{
	/* ���жϱ�־λ */
    PIT_Flag_Clear(pitn);

    /* ��ռ����� */
    PIT_TCTRL(pitn) &= ~PIT_TCTRL_TEN_MASK;
}
