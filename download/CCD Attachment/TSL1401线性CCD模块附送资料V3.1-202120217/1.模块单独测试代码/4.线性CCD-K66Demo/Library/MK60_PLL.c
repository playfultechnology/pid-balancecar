
#include "include.h"
#include "MK60_PLL.h"

uint8_t core_clk;//��λMHZ
uint8_t bus_clk;//��λMHZ

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
     char pll_prdiv;
     char pll_vdiv;

     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                 | SIM_SCGC5_PORTB_MASK
                 | SIM_SCGC5_PORTC_MASK
                 | SIM_SCGC5_PORTD_MASK
                 | SIM_SCGC5_PORTE_MASK );

    u32 temp_reg;

    if(opt!= PLLUNULL )
    {
        //����PLLʱ��
        switch(opt)
        {
        case PLL80:
            pll_prdiv       = 4;
            pll_vdiv        = 0;
            break;
        case PLL90:
            pll_prdiv       = 4;
            pll_vdiv        = 2;
            break;
        case PLL100:
            pll_prdiv       = 4;
            pll_vdiv        = 4;
            break;
        case PLL120:
            pll_prdiv       = 4;
            pll_vdiv        = 8;
            break;
        case PLL130:
            pll_prdiv       = 4;
            pll_vdiv        = 10;
            break;
        case PLL140:
            pll_prdiv       = 4;
            pll_vdiv        = 12;
            break;
        case PLL150:
            pll_prdiv       = 4;
            pll_vdiv        = 14;
            break;
        case PLL160:
            pll_prdiv       = 4;
            pll_vdiv        = 16;
            break;
        case PLL170:
            pll_prdiv       = 4;
            pll_vdiv        = 18;
            break;
        case PLL180:
            pll_prdiv       = 4;
            pll_vdiv        = 20;
            break;
        case PLL200:
            pll_prdiv       = 4;
            pll_vdiv        = 24;
            break;
       case PLL225:
            pll_prdiv       = 4;
            pll_vdiv        = 28;
            break;
        case PLL220:
            pll_prdiv       = 4;
            pll_vdiv        = 29;
            break;
        case PLL230:
            pll_prdiv       = 4;    //�ȶ�
            pll_vdiv        = 30;
            break;
       case PLL235:
            pll_prdiv       = 4;    //���ȶ�
            pll_vdiv        = 31;
            break;
       case PLL237_5:               //�ܲ��ȶ�
            pll_prdiv       = 3;
            pll_vdiv        = 22;
            break;
        default:               break;//(��ʼ��δ�ɹ���ϵͳĬ��ϵͳʱ��Ϊ180M)

        }
    }
    MCG_C1 = MCG_C1_CLKS(2) ;//ѡ���ⲿʱ��


    MCG_C5 = MCG_C5_PRDIV(pll_prdiv);//����Ϊ50M����Ƶ�����ΧҪ��8M~16M ��ʱΪ 50/(prdiv+1)


   temp_reg = FMC_PFAPR;

    //ͨ��M&PFD��λM0PFD����ֹԤȡ����
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
    ///����ϵͳ��Ƶ��
    //MCG=PLL, core = MCG,
    SIM_CLKDIV1 =  SIM_CLKDIV1_OUTDIV1(0)    //core = MCG
                 | SIM_CLKDIV1_OUTDIV2(1)    //bus = MCG/2,
                 | SIM_CLKDIV1_OUTDIV3(2)    //FlexBus = MCG/(2+1)
                 | SIM_CLKDIV1_OUTDIV4(7);   //Flash clock= MCG/8

    //���´�FMC_PFAPR��ԭʼֵ
    FMC_PFAPR = temp_reg;

    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(pll_vdiv);//PLL =  50M/(prdiv+1) * (pll_vdiv+16)

    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
    while (!(MCG_S & MCG_S_LOCK0_MASK)){}; // Wait for LOCK bit to set


    MCG_C1=0x00;

    //�ȴ�ʱ��״̬λ����
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};


    core_clk = 50 * ( pll_vdiv+16 )/2/(pll_prdiv+1);
    bus_clk =core_clk/2;


 /*
        //���ø���ʱ��Ϊ�ں�ʱ��
    SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;
    //��PTA6������ʹ��TRACE_CLKOU����
    PORTA_PCR6 = ( PORT_PCR_MUX(0x7));
    //ʹ��FlexBusģ��ʱ��
    SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
    //��PTA6������ʹ��FB_CLKOUT����
    PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
    */
}