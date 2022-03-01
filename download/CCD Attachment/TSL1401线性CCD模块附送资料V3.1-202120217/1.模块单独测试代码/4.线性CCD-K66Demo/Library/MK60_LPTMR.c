
#include "include.h"
#include "MK60_LPTMR.h"

/*-------------------------------------------------------------------------*
*������: delayms
*��  ��: ��ȷ��ʱ����
*��  ��: ms  :   ��ʱʱ��
*��  ��: ��
*��  ��: delayms(1000);
*ע  ��: ʹ����ʱ��ʱ����ʹ��LPTMR��Ĺ���
-------------------------------------------------------------------------*/
void delayms(u16 ms)
{
  LPTMR_Delayms(ms);
}
/*-------------------------------------------------------------------------*
*������: delayus
*��  ��: ��ȷ��ʱ����
*��  ��: us  :   ��ʱʱ��
*��  ��: ��
*��  ��: delayus(1000);
*ע  ��: ʹ����ʱ��ʱ����ʹ��LPTMR��Ĺ���
-------------------------------------------------------------------------*/
void delayus(u16 us)
{
  LPTMR_delayus(us);
}

/*-------------------------------------------------------------------------*
*������: LPTMR_pulse_init
*��  ��: LPTMR���������ʼ��
*��  ��: LPT0_ALTn:LPTMR��������ܽ�
*        count    :LPTMR����Ƚ�ֵ
*        LPT_CFG  :LPTMR���������ʽ�������ؼ������½��ؼ���
*��  ��: ��
*��  ��:  LPTMR_pulse_init(LPT0_ALT1,32768,LPT_Rising);
-------------------------------------------------------------------------*/
void LPTMR_PulseInit(LPT0_ALTn altn, uint16 count, LPT_CFG cfg)
{

    // ����ģ��ʱ��
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;

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


/*-------------------------------------------------------------------------*
*������: LPTMR_pulse_get
*��  ��: ��ȡLPTMR�������ֵ
*��  ��: �������ֵ
*��  ��: ��
-------------------------------------------------------------------------*/
uint16_t LPTMR_PulseGet(void)
{
    uint16 data;

    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ

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


/*-------------------------------------------------------------------------*
*������: LPTMR_pulse_clean
*��  ��: ���LPTMR�������
*��  ��: ��
*��  ��: ��
*��  ��: ��
-------------------------------------------------------------------------*/
void LPTMR_PulseClean(void)
{
    LPTMR0_CSR  &= ~LPTMR_CSR_TEN_MASK;     //����LPT��ʱ��ͻ��Զ����������ֵ
    LPTMR0_CSR  |= LPTMR_CSR_TEN_MASK;
}

/*-------------------------------------------------------------------------*
*������: LPTMR_delay_ms
*��  ��: LPTMR��ʱ������ms��
*��  ��: ms    ����
*��  ��: ��
*��  ��: LPTMR_delayms(32);     // LPTMR ��ʱ32ms
-------------------------------------------------------------------------*/
void LPTMR_Delayms(uint16 ms)
{
    if(ms == 0)
    {
        return;
    }

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;    //ʹ��LPTģ��ʱ��

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

/*-------------------------------------------------------------------------*
*������: LPTMR_timing_us
*��  ��: LPTMR��ʱ������us��
*��  ��: us        LPTMR��ʱʱ��(0~41942)
*��  ��: ��
*��  ��: LPTMR_delayus(32);     // LPTMR ��ʱ32us
-------------------------------------------------------------------------*/
void LPTMR_delayus(uint16 us)
{

    if(us == 0)
    {
        return;
    }


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //ʹ�� OSCERCLK


    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                        //ʹ��LPTģ��ʱ��

    LPTMR0_CSR = 0x00;                                          //�ȹ���LPT���Զ����������ֵ

    LPTMR0_CMR = (us * 50 + 16) / 32;                 //���ñȽ�ֵ������ʱʱ��

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
                    | LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );

}


//-------------------------------------------------------------------------*
//������: LPTMR��ʱ������ms,��ʱʱ�����Ϊ65534ms��
//��  ��: LPTMR��ʱ����
//��  ��: ��
//��  ��: ��
//��  ��:
/*                 void my_delay(uint32 time)
                    {
                        volatile uint32 i = time;
                        while(i--);

                    }


                    LPTMR_TimeStartms();

                    my_delay(600000);
                    i = LPTMR_TimeGetms();
                    if(i == ~0)
                    {
                        UART_Put_Str("\n��ʱʱ�䳬ʱ");
                    }
                    else
                    {
                        UART_Put_Str("\n��ʱʱ��Ϊ��%dms",i);
                    }
 */

//-------------------------------------------------------------------------*
void LPTMR_TimeStartms(void)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;      //ʹ��LPTģ��ʱ��

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


//-------------------------------------------------------------------------*
//������: LPTMR_time_get_ms
//��  ��: ��ȡLPTMR��ʱʱ�䣨ms��
//��  ��: ��
//��  ��: ��ʱʱ�䣨����ֵΪ ~0 ��ʾ��ʱ��ʱ������ֵ�� 0~ 65534ms ������ ��
//��  ��: �ο� LPTMR_time_start_ms �ĵ�������
//-------------------------------------------------------------------------*
uint32_t LPTMR_TimeGetms(void)
{
    uint32 data;

    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ

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



/*-------------------------------------------------------------------------*
*������: LPTMR_time_close
*��  ��: �ر� LPTMR��ʱ
*��  ��: ��
*��  ��: ��
*��  ��: ��
-------------------------------------------------------------------------*/
void LPTMR_TimeClose()
{
    //�ȹ���LPT���Զ����������ֵ�����������
    LPTMR0_CSR = 0x00;
}



//-------------------------------------------------------------------------*
//������: LPTMR��ʱ������us,��ʱʱ�����Ϊ41942us��
//��  ��: LPTMR��ʱ����
//��  ��: ��
//��  ��: ��
//��  ��:
/*                 void my_delay(uint32 time)
                    {
                        volatile uint32 i = time;
                        while(i--);

                    }

                    uint32 i;
                    LPTMR_TimeStartus();

                    my_delay(5894);

                    i = LPTMR_TimeGetus();
                    if(i == ~0)
                    {
                        UART_Put_Str("\n��ʱʱ�䳬ʱ");
                    }
                    else
                    {
                        UART_Put_Str("\n��ʱʱ��Ϊ��%dus",i);
                    }
 */

//-------------------------------------------------------------------------*
void LPTMR_TimeStartus(void)
{


    OSC_CR |= OSC_CR_ERCLKEN_MASK;                              //ʹ�� OSCERCLK

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                          //ʹ��LPTģ��ʱ��

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


/*-------------------------------------------------------------------------*
*������: LPTMR_TimeGetus
*��  ��: ��ȡLPTMR��ʱʱ�䣨us��
*��  ��: ��
*��  ��: ��ʱʱ�䣨����ֵΪ ~0 ��ʾ��ʱ��ʱ������ֵ�� 0~ 41942us ������ ��
*��  ��: �ο� LPTMR_TimeGetus �ĵ�������
-------------------------------------------------------------------------*/
uint32_t LPTMR_TimeGetus(void)
{
    uint32_t data;

    LPTMR0_CNR = 0;//����д��һ��ֵ������ȡ��ȡ

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
