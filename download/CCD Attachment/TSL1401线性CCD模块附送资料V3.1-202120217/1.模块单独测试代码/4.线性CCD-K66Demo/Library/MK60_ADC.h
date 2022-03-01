
#ifndef __ADC_H__
#define __ADC_H__
/*
                        ADC��Ӧ����
----------------ADC0--------------------ADC1---------------
      ADC0_SE8-----PTB0         ADC1_SE4a-----PTE0
      ADC0_SE9-----PTB1         ADC1_SE5a-----PTE1
      ADC0_SE10----PTA7         ADC1_SE6a-----PTE2
      ADC0_SE11----PTA8         ADC1_SE7a-----PTE3
      ADC0_SE12----PTB2         ADC1_SE8------PTB0
      ADC0_SE13----PTB3         ADC1_SE9------PTB1
      ADC0_SE14----PTC0         ADC1_SE10-----PTB4
      ADC0_SE15----PTC1         ADC1_SE11-----PTB5
                                ADC1_SE12-----PTB6
      ADC0_SE16----ADC0_SE16    ADC1_SE13-----PTB7
                                ADC1_SE14-----PTB10
      ADC0_SE17----PTE24        ADC1_SE15-----PTB11
      ADC0_SE18----PTBE25       ADC1_SE16-----ADC1_SE16
                                ADC1_SE17-----PTA17

      ADC0_DM0-----ADC0_DM0     ADC1_DM0------ADC1_DM0
      ADC0_DM1-----ADC0_DM1     ADC1_DM1------ADC1_DM1
----------------ADC0--------------------ADC1---------------*/

typedef enum
{
    // ---------------------------------ADC0-------------------------
    ADC0_DP0  =0,
    ADC0_DP1  =1,
    PGA0_DP   =2,        //��֧��ADC����Ҫ���� ADC PGA register ���÷Ŵ�����
    ADC0_DP3  =3,
    //���������֧��Bͨ��
    ADC0_SE4b =4,      // PTC2     ��֧�����ADC
    ADC0_SE5b =5,      // PTD1     ��֧�����ADC
    ADC0_SE6b =6,      // PTD5     ��֧�����ADC
    ADC0_SE7b =7,      // PTD6     ��֧�����ADC

    ADC0_SE8  =8,       // PTB0
    ADC0_SE9  =9,       // PTB1
    ADC0_SE10 =10,      // PTA7
    ADC0_SE11 =11,      // PTA8
    ADC0_SE12 =12,      // PTB2
    ADC0_SE13 =13,      // PTB3
    ADC0_SE14 =14,      // PTC0
    ADC0_SE15 =15,      // PTC1
    ADC0_SE16 =16,      // ADC0_SE16
    ADC0_SE17 =17,      // PTE24
    ADC0_SE18 =18,      // PTE25
    ADC0_DM0  =19,      // ADC0_DM0
    ADC0_DM1  =20,      // ADC0_DM1
    ADC0_SE21 =21,      // ����
    ADC0_SE22 =22,      // ����
    DAC0_OUT,       // DAC0��� ��֧��ADC
    RES2,           // ����
    RES3,           // ����
    Temp0_Sensor,   // Temperature Sensor,�ڲ��¶Ȳ���������ADC����
    Bandgap0,       // �¶Ȳ����ṹ��϶��׼Դ   ��֧��ADC
    RES4,           // ����
    VREFH0,         // �ο��ߵ�ѹ,����ADC���� ,�����Ϊ 2^n-1
    VREFL0,         // �ο��͵�ѹ,����ADC���� ,�����Ϊ 0
    Module0_Dis,    // ��֧�� ADC

    // ---------------------------------ADC1-------------------------
    ADC1_DP0  =0,
    ADC1_DP1  =1,
    PGA1_DP   =2,        // ��֧�� ADC
    ADC1_DP3  =3,
    ADC1_SE4a =4,      // PTE0
    ADC1_SE5a =5,      // PTE1
    ADC1_SE6a =6,      // PTE2
    ADC1_SE7a =7,      // PTE3

    ADC1_SE4b = ADC1_SE4a,  // PTC8    Bͨ�� ��֧�����ADC ,���� ADC1_SE4b �����ADC������ᵱ�� ADC1_SE4a ����
    ADC1_SE5b = ADC1_SE5a,  // PTC9    Bͨ�� ��֧�����ADC
    ADC1_SE6b = ADC1_SE6a,  // PTC10   Bͨ�� ��֧�����ADC
    ADC1_SE7b = ADC1_SE7a,  // PTC11   Bͨ�� ��֧�����ADC

    ADC1_SE8  =8,       // PTB0
    ADC1_SE9  =9,       // PTB1
    ADC1_SE10 =10,      // PTB4
    ADC1_SE11 =11,      // PTB5
    ADC1_SE12 =12,      // PTB6
    ADC1_SE13 =13,      // PTB7
    ADC1_SE14 =14,      // PTB10
    ADC1_SE15 =15,      // PTB11
    ADC1_SE16 =16,      // ADC1_SE16
    ADC1_SE17 =17,      // PTA17
    VREF_OUTPUT,    // VREF Output
    ADC1_DM0  =19,       // ADC1_DM0
    ADC1_DM1  =20,       // ADC1_DM1
    RES5,           //����
    RES6,
    DAC1_OUT,
    RES7,           //����
    RES8,
    Temp1_Sensor,
    Bandgap1,       // �¶Ȳ����ṹ��϶��׼Դ   ��֧��ADC
    RES9,
    VREFH1,         // �ο��ߵ�ѹ,����ADC���� ,�����Ϊ 2^n-1
    VREFL1,         // �ο��͵�ѹ,����ADC���� ,�����Ϊ 0
    Module1_Dis,    // ��֧�� ADC

} ADCn_Ch_e;



//����λ��
typedef enum ADC_nbit
{
    ADC_8bit   = 0x00,
    ADC_10bit  = 0x02,
    ADC_12bit  = 0x01,
    ADC_16bit  = 0x03
} ADC_nbit;

void ADC_Init(ADC_Type * adc_n);//��ʼ��ADCģ��

void ADC_Start(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);//����ADCת��

uint16_t ADC_Once(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit);//����ADCת�� ��ȡһ��ADCת��ֵ

uint16_t ADC_Mid(ADC_Type * adc_n,  ADCn_Ch_e adc_ch, ADC_nbit bit);//����ADC3��ת��ȡ�м�ֵ

uint16_t ADC_Ave(ADC_Type * adc_n,  ADCn_Ch_e adc_ch, ADC_nbit bit, uint16_t N);//����ADC N��ת��ȡƽ��ֵ


#endif
