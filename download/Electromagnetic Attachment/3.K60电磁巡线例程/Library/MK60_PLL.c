
#include "include.h"
#include "MK60_PLL.h"

u8 core_clk;//��λMHZ
u8 bus_clk;//��λMHZ

#ifdef MK60FX

/************************************************************************************
*�������ܣ�ʱ�ӳ�ʼ���������趨��Ƶ
*��ڲ�����opt:PLL_?
*�� �� ֵ����
*ʵ    ����pll_init(PLL180)==��ʼ����Ƶ180M
*ע    �⣺�ں�ʱ�ӣ�ϵͳʱ�ӣ�=[�ⲿʱ�ӣ�50M����Ƶ�ʣ�*(pll_vdiv+24)]/(pll_prdiv+1);
           MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/3, Flash clock= MCG/8
************************************************************************************/

void PLL_Init(clk_option opt)
{
     u8 pll_prdiv;
     u8 pll_vdiv;
     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                 | SIM_SCGC5_PORTB_MASK
                 | SIM_SCGC5_PORTC_MASK
                 | SIM_SCGC5_PORTD_MASK
                 | SIM_SCGC5_PORTE_MASK );

    u32 temp_reg;

    if(opt!= PLLUSR )
    {
        //����PLLʱ��
        switch(opt)
        {
        case PLL48:
            pll_prdiv       = 24;
            pll_vdiv        = 0;
            break;
        case PLL50:
            pll_prdiv       = 24;
            pll_vdiv        = 1;
            break;
        case PLL96:
            pll_prdiv       = 24;
            pll_vdiv        = 24;
            break;
        case PLL100:
            pll_prdiv       = 24;
            pll_vdiv        = 26;
            break;
        case PLL110:
            pll_prdiv       = 24;
            pll_vdiv        = 31;
            break;
        case PLL120:
            pll_prdiv       = 19;
            pll_vdiv        = 24;
            break;
        case PLL125:
            pll_prdiv       = 19;
            pll_vdiv        = 26;
            break;
        case PLL130:
            pll_prdiv      = 19;
            pll_vdiv        = 28;
            break;
        case PLL140:
            pll_prdiv      = 14;
            pll_vdiv        = 18;
            break;
        case PLL150:
            pll_prdiv       = 14;
            pll_vdiv        = 21;
            break;
        case PLL160:
            pll_prdiv       = 14;
            pll_vdiv        = 24;
            break;
        case PLL170:
            pll_prdiv       = 14;
            pll_vdiv        = 27;
            break;
        case PLL180:
            pll_prdiv       = 14;
            pll_vdiv        = 30;
            break;
        case PLL200:
            pll_prdiv       = 12;
            pll_vdiv        = 28;
            break;
        case PLL225:
            pll_prdiv       = 11;
            pll_vdiv        = 30;
            break;
        case PLL250:
            pll_prdiv       = 10;
            pll_vdiv        = 31;
            break;
        default:                //(δ�������룬ϵͳĬ��ϵͳʱ��Ϊ100M)
            pll_prdiv       = 24;
            pll_vdiv        = 31;
            break;

        }
    }
    MCG_C1 = MCG_C1_CLKS(2) ;//ѡ���ⲿʱ��


    MCG_C5 = MCG_C5_PRDIV(pll_prdiv);//��Ƶ��СΪ0b����С����


    temp_reg = FMC_PFAPR;

    //ͨ��M&PFD��λM0PFD����ֹԤȡ����
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
    ///����ϵͳ��Ƶ��
    //MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/3, Flash clock= MCG/8
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1)
                 | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);

    //���´�FMC_PFAPR��ԭʼֵ
    FMC_PFAPR = temp_reg;

    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(pll_vdiv);

    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
    while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set


    MCG_C1=0x00;

    //�ȴ�ʱ��״̬λ����
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

    // �����Ѿ������� PEE ģʽ
    uint32_t mcgout_clk_mhz = 50 * ((MCG->C6 & MCG_C6_VDIV_MASK) + 16) / ((MCG->C5 & MCG_C5_PRDIV_MASK) + 1)/2;
    core_clk = mcgout_clk_mhz / ((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
    bus_clk = mcgout_clk_mhz / (((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV2_SHIFT) & 0x0f) + 1);
}
#else

/************************************************************************************
*�������ܣ�ʱ�ӳ�ʼ���������趨��Ƶ
*��ڲ�����opt:PLL_?
*�� �� ֵ����
*ʵ    ����pll_init(PLL180)==��ʼ����Ƶ180M
*ע    �⣺�ں�ʱ�ӣ�ϵͳʱ�ӣ�=[�ⲿʱ�ӣ�50M����Ƶ�ʣ�*(pll_vdiv+24)]/(pll_prdiv+1);
           MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/3, Flash clock= MCG/8
************************************************************************************/
void PLL_Init(clk_option opt)
{
     u8 pll_prdiv;
     u8 pll_vdiv;
     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                 | SIM_SCGC5_PORTB_MASK
                 | SIM_SCGC5_PORTC_MASK
                 | SIM_SCGC5_PORTD_MASK
                 | SIM_SCGC5_PORTE_MASK );

    u32 temp_reg;

    if(opt!= PLLUSR )
    {
        //����PLLʱ��
        switch(opt)
        {
        case PLL48:
            pll_prdiv       = 24;
            pll_vdiv        = 0;
            break;
        case PLL50:
            pll_prdiv       = 24;
            pll_vdiv        = 1;
            break;
        case PLL96:
            pll_prdiv       = 24;
            pll_vdiv        = 24;
            break;
        case PLL100:
            pll_prdiv       = 24;
            pll_vdiv        = 26;
            break;
        case PLL110:
            pll_prdiv       = 24;
            pll_vdiv        = 31;
            break;
        case PLL120:
            pll_prdiv       = 19;
            pll_vdiv        = 24;
            break;
        case PLL125:
            pll_prdiv       = 19;
            pll_vdiv        = 26;
            break;
        case PLL130:
            pll_prdiv      = 19;
            pll_vdiv        = 28;
            break;
        case PLL140:
            pll_prdiv      = 14;
            pll_vdiv        = 18;
            break;
        case PLL150:
            pll_prdiv       = 14;
            pll_vdiv        = 21;
            break;
        case PLL160:
            pll_prdiv       = 14;
            pll_vdiv        = 24;
            break;
        case PLL170:
            pll_prdiv       = 14;
            pll_vdiv        = 27;
            break;
        case PLL180:
            pll_prdiv       = 14;
            pll_vdiv        = 30;
            break;
        case PLL200:
            pll_prdiv       = 12;
            pll_vdiv        = 28;
            break;
        case PLL225:
            pll_prdiv       = 11;
            pll_vdiv        = 30;
            break;
        case PLL250:
            pll_prdiv       = 10;
            pll_vdiv        = 31;
            break;
        default:
            break;//(��ʼ��δ�ɹ���ϵͳĬ��ϵͳʱ��Ϊ100M)

        }
    }
    MCG_C1 = MCG_C1_CLKS(2) ;//ѡ���ⲿʱ��


    MCG_C5 = MCG_C5_PRDIV(pll_prdiv);//��Ƶ��СΪ0b����С����


   temp_reg = FMC_PFAPR;

    //ͨ��M&PFD��λM0PFD����ֹԤȡ����
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
    ///����ϵͳ��Ƶ��
    //MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/3, Flash clock= MCG/8
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1)
                 | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);

    //���´�FMC_PFAPR��ԭʼֵ
    FMC_PFAPR = temp_reg;

    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(pll_vdiv);

    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
    while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set


    MCG_C1=0x00;

    //�ȴ�ʱ��״̬λ����
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

    // �����Ѿ������� PEE ģʽ
    uint32_t mcgout_clk_mhz = 50 * ((MCG->C6 & MCG_C6_VDIV_MASK) + 24) / ((MCG->C5 & MCG_C5_PRDIV_MASK) + 1);
    core_clk = mcgout_clk_mhz / ((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
    bus_clk = mcgout_clk_mhz / (((SIM->CLKDIV1 >> SIM_CLKDIV1_OUTDIV2_SHIFT) & 0x0f) + 1);
}
#endif

