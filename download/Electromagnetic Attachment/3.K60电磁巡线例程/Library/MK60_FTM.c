#include "include.h"
#include "MK60_FTM.h"

#ifdef MK60FX
/* �����ĸ�ָ�����鱣�� FTMn_e �ĵ�ַ */
FTM_MemMapPtr FTMN[4] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR, FTM3_BASE_PTR};
/* FTM���PWMʱ�����ڼ���ֵ */
static uint32 ftm_period[4];
#else
/* ��������ָ�����鱣�� FTMn_e �ĵ�ַ */
FTM_MemMapPtr FTMN[3] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR};
/* FTM���PWMʱ�����ڼ���ֵ */
static uint32 ftm_period[3];
#endif


/* FTM���ų�ʼ�� */
static void FTM_PinInit(FTMn_e ftmn, FTM_CHn_e ch, uint8_t mode);


/*------------------------------------------------------------------------------------------------------
����    ����FTM_PwmInit
����    �ܡ���ʼ��FTMģʽPWMͨ��
����    ����ftmn  :  ģ����FTM0,FTM1��FTM2
����    ����ch    :  ͨ���� ��Ӧ���Ųο�ftm.h�ļ�
����    ����freq  �� ����PWM��Ƶ��
����    ����duty  �� ����PWM��ռ�ձ�
���� �� ֵ����
��ʵ    ����FTM_PwmInit(FTM0, FTM_CH0, 10000, 500); //Ƶ��10KHZ��ռ�ձ�Ϊ�ٷ�֮��500/FTM_PRECISON *100��;
--------------------------------------------------------------------------------------------------------*/
void FTM_PwmInit(FTMn_e ftmn, FTM_CHn_e ch, uint16_t freq, uint16_t duty)
{

    /* ����FTMʱ�����ö˿ڸ��� */
    FTM_PinInit(ftmn, ch, 1);

    /* ����FTMģ������ʱ�� */
    uint32_t clk_hz = (bus_clk * 1000 * 1000) ;
    uint16_t mod = (clk_hz >> 16 ) / freq ;
    uint8_t ps = 0;
    while((mod >> ps) >= 1)             // �� (mod >> ps) < 1 ���˳� while ѭ�� ������ PS ����Сֵ
    {
        ps++;
    }

    if(ps>0x07) return ;               // ���ԣ� PS ���Ϊ 0x07 ��������ֵ���� PWMƵ�����ù��ͣ��� Bus Ƶ�ʹ���
    mod = (clk_hz >> ps) / freq;        // �� MOD ��ֵ
    ftm_period[ftmn]=mod;
    uint16_t cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    FTM_SC_REG(FTMN[ftmn])   =  FTM_SC_PS(ps) | FTM_SC_CLKS(1);

    /******************** ѡ�����ģʽΪ ���ض���PWM *******************/
    //ͨ��״̬���ƣ�����ģʽ��ѡ�� ���ػ��ƽ
    FTM_CnSC_REG(FTMN[ftmn], ch) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMN[ftmn], ch)  = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
   // FTM_MODE_REG(FTMN[ftmn])&=~1;
    FTM_MOD_REG(FTMN[ftmn])   = mod;                        //ģ��, EPWM������Ϊ ��MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMN[ftmn]) = 0;                          //��������ʼ��ֵ�����������ȣ�(CnV - CNTIN).
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;
    FTM_CNT_REG(FTMN[ftmn])   = 0;                          //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��
}

/*------------------------------------------------------------------------------------------------------
����    ����FTM_PwmDuty
����    �ܡ����PWM��
����    ����ftmn  :  ģ����FTM0,FTM1��FTM2
����    ����ch    :  ͨ���� ��Ӧ���Ųο�ftm.h�ļ�
����    ����duty  �� ����PWM��ռ�ձ�
���� �� ֵ����
��ʵ    ����FTM_PwmDuty(FTM0, FTM_CH0, 500); //ռ�ձ�Ϊ�ٷ�֮��500/FTM_PRECISON *100��;
��ע�����
--------------------------------------------------------------------------------------------------------*/
void FTM_PwmDuty(FTMn_e ftmn, FTM_CHn_e ch, u16 duty)
{
    uint32_t mod = ftm_period[ftmn];
    uint16_t cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    // ����FTMͨ��ֵ
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;

}


//////////////////////////////// ����Ϊ�������� //////////////////////////////////////////



/*------------------------------------------------------------------------------------------------------
����    ����FTM_ABInit
����    �ܡ���ʼ��FTMģʽ��������ģʽ
����    ����ftmn  :  ģ����FTM1��FTM2
���� �� ֵ����
��ʵ    ����FTM_ABInit(FTM1); //FTM1Ϊ��������ģʽ
��ע�����ʹ�ùܽ���ftm.h�ļ��鿴
--------------------------------------------------------------------------------------------------------*/
void FTM_ABInit(FTMn_e ftmn)
{
   /* ����FTMʱ�����ö˿ڸ��� */
    FTM_PinInit(ftmn, FTM_CH0, 0);

    FTM_MODE_REG(FTMN[ftmn])  |=    (0
                                     | FTM_MODE_WPDIS_MASK  //д������ֹ
                                     //| FTM_MODE_FTMEN_MASK   //ʹ�� FTM
                                    );
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADMODE_MASK
                                     );
    FTM_CNTIN_REG(FTMN[ftmn])   = 0;
    FTM_MOD_REG(FTMN[ftmn])     = FTM_MOD_MOD_MASK;
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADEN_MASK
                                     );
    FTM_MODE_REG(FTMN[ftmn])  |= FTM_QDCTRL_QUADEN_MASK;;
    FTM_CNT_REG(FTMN[ftmn])     = 0;                    //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��

}


/*------------------------------------------------------------------------------------------------------
����    ����FTM_ABGet
����    �ܡ���ȡ��������ļ�����ֵ
����    ����ftmn  :  ģ����FTM1��FTM2
���� �� ֵ������ֵ
��ʵ    ����FTM_ABGet(FTM1); //FTM1��������ֵ
��ע�����ʹ�ùܽ���ftm.h�ļ��鿴
--------------------------------------------------------------------------------------------------------*/
short FTM_ABGet(FTMn_e ftmn)
{
    short val;

    val = FTM_CNT_REG(FTMN[ftmn]);

    FTM_CNT_REG(FTMN[ftmn]) = 0;

    return val;
}



/*------------------------------------------------------------------------------------------------------
����    ����FTM_PinInit
����    �ܡ���ʼ��FTM�ܽ�ͨ��  �ڲ�����
����    ����ftmn  :  ģ����FTM0,FTM1��FTM2
����    ����ch    :  ͨ���� ��Ӧ���Ųο�ftm.h�ļ�
����    ����mode  �� 1��PWM�ܽ�   0����������ܽ�
���� �� ֵ����
��ʵ    ����FTM_PwmInit(FTM0, FTM_CH0, 1); //��ʼ��FTM0��FTM_CH0ͨ��ΪPWMģʽ
��ע�����
--------------------------------------------------------------------------------------------------------*/
static void FTM_PinInit(FTMn_e ftmn, FTM_CHn_e ch, uint8_t mode)
{
    if(mode)
    {
        switch(ftmn)
        {
          case FTM0:
            SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;       //ʹ��FTM0ʱ��

            switch(ch)
            {
              case FTM_CH0:
                if(FTM0_CH0 == PTC1)
                {
                    PORTC_PCR1= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH0 == PTA3)
                {
                    PORTA_PCR3= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH1:
                if(FTM0_CH1 == PTC2)
                {
                    // port_init(FTM0_CH1, ALT4);
                    PORTC_PCR2= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH1 == PTA4)
                {
                    PORTA_PCR4= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH2:
                if(FTM0_CH2 == PTC3)
                {
                    PORTC_PCR3= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH2 == PTA5)
                {
                    PORTA_PCR5= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH3:
                if(FTM0_CH3 == PTC4)
                {
                    PORTC_PCR4= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH3 == PTA6)
                {
                    PORTA_PCR6= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH4:
                if(FTM0_CH4 == PTD4)
                {
                    PORTD_PCR4= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH4 == PTA7)
                {
                    PORTA_PCR7= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH5:
                if(FTM0_CH5 == PTD5)
                {
                    PORTD_PCR5= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH5 == PTA0)
                {
                    PORTA_PCR0= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH6:
                if(FTM0_CH6 == PTD6)
                {
                    PORTD_PCR6= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH6 == PTA1)
                {
                    PORTA_PCR1= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH7:
                if(FTM0_CH7 == PTD7)
                {
                    PORTD_PCR7= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH7 == PTA2)
                {
                    PORTA_PCR2= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;
              default:
                return;
            }
            break;

          case FTM1:
            SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //ʹ��FTM1ʱ��
            switch(ch)
            {
              case FTM_CH0:
                if(FTM1_CH0 == PTA8)
                {
                    PORTA_PCR8= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH0 == PTA12)
                {
                    PORTA_PCR12= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH0 == PTB0)
                {
                    PORTB_PCR0= PORT_PCR_MUX(3);
                }
                break;


              case FTM_CH1:
                if(FTM1_CH1 == PTA9)
                {
                    PORTA_PCR9= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH1 == PTA13)
                {
                    PORTA_PCR13= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH1 == PTB1)
                {
                    PORTB_PCR1= PORT_PCR_MUX(3);
                }
                break;

              default:
                return;
            }
            break;

          case FTM2:
            SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //ʹ��FTM2ʱ��
            switch(ch)
            {
              case FTM_CH0:
                if(FTM2_CH0 == PTA10)
                {
                    PORTA_PCR10= PORT_PCR_MUX(3);
                }
                else if(FTM2_CH0 == PTB18)
                {
                    PORTB_PCR18= PORT_PCR_MUX(3);
                }
                break;

              case FTM_CH1:
                if(FTM2_CH1 == PTA11)
                {
                    PORTA_PCR11= PORT_PCR_MUX(3);
                }
                else if(FTM2_CH1 == PTB19)
                {
                    PORTB_PCR19= PORT_PCR_MUX(3);
                }
                break;

              default:
                return;
            }
            break;
          default:
            break;

            #ifdef MK60FX
      case FTM3:
        SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

        switch(ch)
        {
          case FTM_CH0:
            if(FTM3_CH0 == PTE5)
            {
                PORTE_PCR5=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH0 == PTD0)
            {
                PORTD_PCR0=PORT_PCR_MUX(4);
            }
            break;

          case FTM_CH1:
            if(FTM3_CH1 == PTE6)
            {
                PORTE_PCR6=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH1 == PTD1)
            {
                PORTD_PCR1=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH2:
            if(FTM3_CH2 == PTE7)
            {
                PORTE_PCR7=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH2 == PTD2)
            {
                PORTD_PCR2=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH3:
            if(FTM3_CH3 == PTE8)
            {
                PORTE_PCR8=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH3 == PTD3)
            {
                PORTD_PCR3=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH4:
            if(FTM3_CH4 == PTE9)
            {
                PORTE_PCR9=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH4 == PTC8)
            {
                PORTC_PCR8=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH5:
            if(FTM3_CH5 == PTE10)
            {
                PORTE_PCR10=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH5 == PTC9)
            {
                PORTC_PCR9=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH6:
            if(FTM3_CH6 == PTE11)
            {
                PORTE_PCR11=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH6 == PTC10)
            {
                PORTC_PCR10=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH7:
            if(FTM3_CH7 == PTE12)
            {
                PORTE_PCR12=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH7 == PTC11)
            {
                PORTC_PCR11=PORT_PCR_MUX(4);
            }
            break;
          default:  return;

        }
#endif
        }

    }
    else
    {

        switch(ftmn)
        {

          case FTM1:
            SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //ʹ��FTM1ʱ��
            if(FTM1_QDPHA == PTA8)                  //�ܽŸ���
            {
                PORTA_PCR8= PORT_PCR_MUX(6);
            }
            else if(FTM1_QDPHA == PTA12)
            {
                PORTA_PCR12= PORT_PCR_MUX(7);
            }
            else if(FTM1_QDPHA == PTB0)
            {
                PORTB_PCR0= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }

            if(FTM1_QDPHB == PTA9)
            {
                PORTA_PCR9= PORT_PCR_MUX(6);
            }
            else if(FTM1_QDPHB == PTA13)
            {
                PORTA_PCR13= PORT_PCR_MUX(7);
            }
            else if(FTM1_QDPHB == PTB1)
            {
                PORTB_PCR1= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }
            break;


          case FTM2:
            SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //ʹ��FTM2ʱ��
            if(FTM2_QDPHA == PTA10)                  //�ܽŸ���
            {
                PORTA_PCR10= PORT_PCR_MUX(6);
            }
            else if(FTM2_QDPHA == PTB18)
            {
                PORTB_PCR18= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }

            if(FTM2_QDPHB == PTA11)                  //�ܽŸ���
            {
                PORTA_PCR11= PORT_PCR_MUX(6);
            }
            else if(FTM2_QDPHB == PTB19)
            {
                PORTB_PCR19= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }break;
        }

    }
}

