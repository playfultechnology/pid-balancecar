#include "include.h"
#include "MK60_CMT.h"

/* ���ڱ���CMT�����������ڼ���ֵ �ڲ�ʹ�� */
static uint16_t cmt_period;

/*------------------------------------------------------------------------------------------------------
����    ����CMT_PwmInit
����    �ܡ���ʼ��CMTģʽ
����    ����freq  �� ����PWM��Ƶ��
����    ����duty  �� ����PWM��ռ�ձ�
���� �� ֵ����
��ʵ    ����CMT_PwmInit(50, 50); //Ƶ��50HZ��ռ�ձ�Ϊ�ٷ�֮��50/CMT_PRECISON *100��;
��ע�����CMT���PWMֻ����PD7�ܽ�
--------------------------------------------------------------------------------------------------------*/
void CMT_PwmInit(uint16_t freq, uint16_t duty)
{
    uint32_t temp_clk;
    uint32_t temp_high_num, temp_low_num;
    uint32_t temp_div;
    //ʹ��ʱ��
    SIM->SCGC4 |= SIM_SCGC4_CMT_MASK;
    //���ø��ù���ΪCMT����
    PORTD_PCR7= PORT_PCR_MUX(2);
    //ʹ�����
    CMT->OC |= CMT_OC_IROPEN_MASK;

#ifdef MK60FX
        //����CMTģ���̶��˷�Ƶ�����������㣬���ں���ʹ��
    uint32_t mcgout_clk_mhz = 50 * ((MCG->C6 & MCG_C6_VDIV_MASK) + 16) / ((MCG->C5 & MCG_C5_PRDIV_MASK) + 1)/2;
    uint32_t core_clk_mhz = mcgout_clk_mhz / ((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
    uint32_t bus_clk_mhz = mcgout_clk_mhz / (((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV2_SHIFT) & 0x0f) + 1);
    temp_clk = bus_clk_mhz*1000*1000/8;
#else
    //����CMTģ���̶��˷�Ƶ�����������㣬���ں���ʹ��
    uint32_t mcgout_clk_mhz = 50 * ((MCG->C6 & MCG_C6_VDIV_MASK) + 24) / ((MCG->C5 & MCG_C5_PRDIV_MASK) + 1);
    uint32_t core_clk_mhz = mcgout_clk_mhz / ((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
    uint32_t bus_clk_mhz = mcgout_clk_mhz / (((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV2_SHIFT) & 0x0f) + 1);
    temp_clk = bus_clk_mhz*1000*1000/8;
#endif
    //������ѷ�Ƶ
    temp_div = temp_clk/freq;
    temp_div = (temp_div>>16);
    if(temp_div>0x0f)   temp_div = 0x0f;

    //���÷�Ƶ
    CMT->PPS = CMT_PPS_PPSDIV(temp_div);

    //����һ��������Ҫ�����Ĵ���

    cmt_period = temp_clk/(temp_div+1)/freq;

    //����ߵ͵�ƽ�ļ�������
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);

    //���õ͵�ƽʱ��
    temp_low_num--;
    CMT->CMD1 = temp_low_num >> 8;
    CMT->CMD2 = (uint8)temp_low_num;

    //���øߵ�ƽʱ��
    CMT->CMD3 = temp_high_num >> 8;
    CMT->CMD4 = (uint8)temp_high_num;

    //����ģʽ��ʹ��CMTģ��
    CMT->MSC = CMT_MSC_BASE_MASK | CMT_MSC_MCGEN_MASK;
}


/*------------------------------------------------------------------------------------------------------
����    ����CMT_PwmDuty
����    �ܡ���ʼ��CMTģʽ
����    ����duty  �� ����PWM��ռ�ձ�
���� �� ֵ����
��ʵ    ����CMT_PwmInit(50); //ռ�ձ�Ϊ�ٷ�֮��50/CMT_PRECISON *100��;
��ע�����
--------------------------------------------------------------------------------------------------------*/
void CMT_PwmDuty(uint16_t duty)
{

    uint32_t temp_high_num, temp_low_num;

    //����ߵ͵�ƽ�ļ�������
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);

    //���õ͵�ƽʱ��
    temp_low_num--;
    CMT->CMD1 = temp_low_num >> 8;
    CMT->CMD2 = (uint8)temp_low_num;

    //���øߵ�ƽʱ��
    CMT->CMD3 = temp_high_num >> 8;
    CMT->CMD4 = (uint8)temp_high_num;

}


