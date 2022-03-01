
#include "include.h"
#include "MK60_GPIO.h"

/* �������ָ�����鱣�� GPIOX �ĵ�ַ */
GPIO_MemMapPtr GPIOX[5] = {PTA_BASE_PTR, PTB_BASE_PTR, PTC_BASE_PTR, PTD_BASE_PTR, PTE_BASE_PTR};
PORT_MemMapPtr PORTX[5] = {PORTA_BASE_PTR, PORTB_BASE_PTR, PORTC_BASE_PTR, PORTD_BASE_PTR, PORTE_BASE_PTR};

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_Init
����    �ܡ���ʼ��GPIO ������GPIOģʽ
����    ����ptx_n �� Ҫ��ʼ����GPIO�� ��common.h�ж���
����    ����dir   �� GPIO��������ã� ������GPIO.h��
����    ����data  �� GPIOĬ��״̬  1���ߵ�ƽ  0���͵�ƽ
���� �� ֵ����
��ʵ    ����GPIO_Init(PTA17, GPO, 1); //����PTA17Ϊ���ģʽ ������Ϊ�ߵ�ƽ
��ע�����
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinInit(PTXn_e ptx_n, GPIO_CFG dir, uint8_t data)
{

    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* ʹ�ܶ˿�ʱ�� */
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK << ptx);

    /* ���֮ǰ�ĸ��ù��� */
    PORTX[ptx]->PCR[ptn] &= ~(uint32)PORT_PCR_MUX_MASK;

    /* ���ø��ù���ΪGPIO����ͨIO�� */
    PORTX[ptx]->PCR[ptn] |= PORT_PCR_MUX(1);

    /* ����GPIOģʽ */
    PORTX[ptx]->PCR[ptn] |= dir;

    /* ����GPIO���� */
    if(dir)
    {
        GPIOX[ptx]->PDDR |= (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);
    }

    /* ���ö˿�Ĭ��״̬ */
    if(data)
    {
        GPIOX[ptx]->PDOR |=  (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDOR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_PinSetDir
����    �ܡ�����IO�����뻹�����
����    ����ptx_n �� IO��
����    ����dir   �� GPIO����   1�����  0������
���� �� ֵ����
��ʵ    ����GPIO_PinSetDir(PTA17, 1); //����PTA17���
��ע�����ע��Ҫʹ��GPIO��ʼ������
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinSetDir(PTXn_e ptx_n, uint8_t dir)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* ����GPIO���� */
    if(dir)
    {
        GPIOX[ptx]->PDDR |= (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_PinWrite
����    �ܡ�����IO�����
����    ����ptx_n �� Ҫ��ʼ����GPIO�� ��common.h�ж���
����    ����data  �� GPIO���״̬  1���ߵ�ƽ  0���͵�ƽ
���� �� ֵ����
��ʵ    ����GPIO_PinWrite(PTA17, 1); //����PTA17����ߵ�ƽ
��ע�����ע��Ҫʹ��GPIO��ʼ������
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinWrite(PTXn_e ptx_n, uint8_t data)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* ���ö˿�Ĭ��״̬ */
    if(data)
    {
        GPIOX[ptx]->PDOR |=  (uint32)(1<<ptn);
    }
    else
    {
        GPIOX[ptx]->PDOR &= ~(uint32)(1<<ptn);
    }
}

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_PinRead
����    �ܡ���ȡIO�ڵ�ƽ
����    ����ptx_n �� Ҫ��ʼ����GPIO�� ��common.h�ж���
���� �� ֵ��1�� �ߵ�ƽ   0���͵�ƽ
��ʵ    ����GPIO_PinRead(PTA17); //��ȡPTA17�ܽŵ�ƽ
��ע�����ע��Ҫʹ��GPIO��ʼ������
--------------------------------------------------------------------------------------------------------*/
uint8_t GPIO_PinRead(PTXn_e ptx_n)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    return ( (GPIOX[ptx]->PDIR >> ptn) & 0x1 );
}

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_PinReverse
����    �ܡ�GPIOȡ��
����    ����ptx_n �� Ҫ��ʼ����GPIO�� ��common.h�ж���
���� �� ֵ����
��ʵ    ����GPIO_PinReverse(PTA17); //ȡ��PTA17�ܽŵ�ƽ
��ע�����ע��Ҫʹ��GPIO��ʼ������
--------------------------------------------------------------------------------------------------------*/
void GPIO_PinReverse(PTXn_e ptx_n)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    GPIOX[ptx]->PTOR ^= (1<<ptn);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************************************
                                                    gpio�ⲿ�жϺ���
**************************************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*------------------------------------------------------------------------------------------------------
����    ����GPIO_PinReverse
����    �ܡ�GPIOȡ��
����    ����ptx_n �� Ҫ��ʼ����GPIO�� ��common.h�ж���
���� �� ֵ����
��ʵ    ����GPIO_ExtiInit(PTA17, rising_down); //PTA17�ܽ������ش����ж�
��ע�����ע����Ҫʹ��NVIC_SetPriority������PIT�ж����ȼ�   NVIC_EnableIRQ��ʹ���ж�
��ע�����
��ע��������ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
��ע�����NVIC_SetPriority(PORTA_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1,2));
��ע�����NVIC_EnableIRQ(PORTA_IRQn);			         //ʹ��PORTA_IRQn���ж�
--------------------------------------------------------------------------------------------------------*/
void GPIO_ExtiInit(PTXn_e ptx_n, exti_cfg cfg)
{
    uint8_t ptx,ptn;

    ptx = PTX(ptx_n);
    ptn = PTn(ptx_n);

    /* ʹ�ܶ˿�ʱ�� */
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK << ptx);

    //����жϱ�־λ
    PORTX[ptx]->ISFR = (uint32)1<<ptn;

    /* ���ö˿ڹ��� */
    PORT_PCR_REG(PORTX[ptx], ptn) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(cfg & 0x7f ) | PORT_PCR_PE_MASK | ((cfg & 0x80 ) >> 7);

    //���ö˿�Ϊ����
    GPIOX[ptx]->PDDR &= ~(uint32)(1<<ptn);

}
