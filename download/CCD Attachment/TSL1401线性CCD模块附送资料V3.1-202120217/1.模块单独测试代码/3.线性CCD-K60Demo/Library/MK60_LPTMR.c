
#include "include.h"
#include "MK60_LPTMR.h"

/**************************************************************************
*�������ܣ�ms��ʱ����
*��ڲ�����ms:��ʱms��
*�� �� ֵ����
*ʵ    ����delayms(50)==��ʱ50ms
*ע    �⣺ʹ����ʱ��ʱ����ʹ��LPTMR��Ĺ���
**************************************************************************/
void delayms(u16 ms)
{
  LPTMR_Delayms(ms);
}

/**************************************************************************
*�������ܣ�LPTMR���������ʼ��
*��ڲ�����LPT0_ALTn:LPTMR��������ܽ�
*          count    :LPTMR����Ƚ�ֵ
*          LPT_CFG  :LPTMR���������ʽ�������ؼ������½��ؼ���
*�� �� ֵ����
*ʵ    ����LPTMR_pulse_init(LPT0_ALT1,32768,LPT_Rising)==A19,�������������
*ע    �⣺ʹ����ʱ��ʱ����ʹ��LPTMR��Ĺ���
**************************************************************************/
void LPTMR_PulseInit(LPT0_ALTn altn, uint16 count, LPT_CFG cfg)
{

    // ����ģ��ʱ��
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

    //��������ܽ�
    if(altn == LPT0_ALT1)
    {
         PORTA_PCR19= PORT_PCR_MUX(6);
    }
    else if(altn == LPT0_ALT2)
    {
         PORTC_PCR5= PORT_PCR_MUX(4);
    }
    else                                    //�����ܷ����¼�
    {
       ;
    }

    // ��״̬�Ĵ���
    LPTMR0_CSR = 0x00;                                          //�ȹ���LPT��������������ʱ�ӷ�Ƶ,��ռ���ֵ��


    //ѡ��ʱ��Դ
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      //| LPTMR_PSR_PRESCALE(1)           //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );


    // �����ۼӼ���ֵ
    LPTMR0_CMR  =   LPTMR_CMR_COMPARE(count);                   //���ñȽ�ֵ
    LPTMR_Flag_Clear();
    // �ܽ����á�ʹ���ж�
    LPTMR0_CSR  =  (0
                    | LPTMR_CSR_TPS(altn)       // ѡ������ܽ� ѡ��
                    | LPTMR_CSR_TMS_MASK        // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    | ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    | LPTMR_CSR_TIE_MASK        //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );
}

/**************************************************************************
*�������ܣ���ȡLPTMR�������ֵ
*��ڲ�������
*�� �� ֵ��data���������ֵ
*ʵ    ������
**************************************************************************/
uint16_t LPTMR_PulseGet(void)
{
    uint16 data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //�Ѿ������
    {

        data = ~0;                          //���� 0xffffffff ��ʾ����
    }
    else
    {
        data = LPTMR0_CNR;
    }
    return data;
}

/**************************************************************************
*�������ܣ����LPTMR�������
*��ڲ�������
*�� �� ֵ����
*ʵ    ������
**************************************************************************/
void LPTMR_PulseClean(void)
{
    LPTMR0_CSR  &= ~LPTMR_CSR_TEN_MASK;     //����LPT��ʱ��ͻ��Զ����������ֵ
    LPTMR0_CSR  |= LPTMR_CSR_TEN_MASK;
}

/**************************************************************************
*�������ܣ�LPTMR��ʱ������ms��
*��ڲ�����ms����Ҫ��ʱ��ms��
*�� �� ֵ����
*ʵ    ����LPTMR_delay_ms(50)==��ʱ50ms
**************************************************************************/
void LPTMR_Delayms(uint16 ms)
{
    if(ms == 0)
    {
        return;
    }

    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;    //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                      //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = ms;                        //���ñȽ�ֵ������ʱʱ��

    //ѡ��ʱ��Դ
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      //| LPTMR_PSR_PRESCALE(1)           //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );

    //ʹ�� LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // ѡ������ܽ� ѡ��
                    //| LPTMR_CSR_TMS_MASK      // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    //| LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );

    while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)); //�ȴ��Ƚ�ֵ�����ֵ��ȣ���ʱ�䵽��

    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;          //����Ƚϱ�־λ

    return;
}

void LPTMR_TimeStartms(void)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;                        //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                      //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = ~0;                        //���ñȽ�ֵ������ʱʱ��

    //ѡ��ʱ��Դ
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      //| LPTMR_PSR_PRESCALE(1)           //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );

    //ʹ�� LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // ѡ������ܽ� ѡ��
                    //| LPTMR_CSR_TMS_MASK      // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    //| LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );

}

/**************************************************************************
*�������ܣ���ȡLPTMR��ʱʱ�䣨ms��
*��ڲ�������
*�� �� ֵ��data����ʱʱ��~0==��ʱ�����0~65535֮��������
*ʵ    ������
**************************************************************************/
uint32_t LPTMR_TimeGetms(void)
{
    uint32 data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //�Ѿ������
    {

        data = ~0;                          //���� 0xffffffff ��ʾ����
    }
    else
    {
        data = LPTMR0_CNR;                  //���ؼ�ʱʱ��(��ֵ���Ϊ 0xffff)
    }

    return data;
}

/**************************************************************************
*�������ܣ��ر� LPTMR��ʱ
*��ڲ�������
*�� �� ֵ����
*ʵ    ������
**************************************************************************/
void LPTMR_TimeClose()
{
    LPTMR0_CSR = 0x00; //�ȹ���LPT���Զ����������ֵ�����������
}

/**************************************************************************
*�������ܣ�����us��ʱ
*��ڲ�������
*�� �� ֵ����
*ʵ    ������
**************************************************************************/
void LPTMR_TimeStartus(void)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //ʹ�� OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;                        //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                                          //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = ~0;                                            //���ñȽ�ֵΪ���ֵ

    //ѡ��ʱ��Դ
    LPTMR0_PSR  =   ( 0
                      | LPTMR_PSR_PCS(3)          //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      //| LPTMR_PSR_PBYP_MASK     //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      | LPTMR_PSR_PRESCALE(4)     //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );

    //ʹ�� LPT
    LPTMR0_CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // ѡ������ܽ� ѡ��
                    //| LPTMR_CSR_TMS_MASK      // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    //| LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );
}

/**************************************************************************
*�������ܣ���ȡLPTMR��ʱʱ�䣨us��
*��ڲ�������
*�� �� ֵ��data����ʱʱ��~0==��ʱ�����0~65535֮��������
*ʵ    ������
**************************************************************************/
uint32_t LPTMR_TimeGetus(void)
{
    uint32_t data;
#ifdef MK60FX
    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ
#endif
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //�Ѿ������
    {
        data = ~0;                          //���� 0xffffffff ��ʾ����
    }
    else
    {
        data = (LPTMR0_CNR * 32) / 50; //���е�λ����
    }

    return data;
}
