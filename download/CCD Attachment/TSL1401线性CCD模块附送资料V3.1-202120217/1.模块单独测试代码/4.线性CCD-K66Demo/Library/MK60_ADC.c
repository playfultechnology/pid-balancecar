
#include "include.h"
#include "MK60_ADC.h"

/*------------------------------------------------------------------------------------------------------
����    ����ADC_Init
����    �ܡ���ʼ��ADCģʽ
����    ����ADCn_e�� Ҫ��ʼ����ADCģ�飬 ADC0  ADC1
���� �� ֵ����
��ʵ    ����ADC_Init(ADC0); //��ʼ��ADC0ģ��
��ע�����
--------------------------------------------------------------------------------------------------------*/
void ADC_Init(ADC_Type * adc_n)
{
    if(adc_n==ADC0)
    {
        SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK );        //����ADC0ʱ��
        SIM_SOPT7 &= ~(SIM_SOPT7_ADC0ALTTRGEN_MASK  | SIM_SOPT7_ADC0PRETRGSEL_MASK);
        SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0);
    }
    else
    {
        SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );
        SIM_SOPT7 &= ~(SIM_SOPT7_ADC1ALTTRGEN_MASK  | SIM_SOPT7_ADC1PRETRGSEL_MASK) ;
        SIM_SOPT7 |= SIM_SOPT7_ADC1TRGSEL(0);
    }


 }

/*------------------------------------------------------------------------------------------------------
����    ����ADC_Start
����    �ܡ�����ADCת��
����    ����adc_n ��  ģ����ADC0��ADC1
����    ����adc_ch��  ADCͨ�����
����    ����bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
���� �� ֵ����
��ʵ    ����ADC_Init(ADC0,ADC1_SE8,ADC_12bit); //ͨ�����Ϊ ADC1_SE8������
��ע�����ʹ��ǰӦ��ʼ����ӦADCģ��
--------------------------------------------------------------------------------------------------------*/
void ADC_Start(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
{
   if(adc_n==ADC0)
   {
     ADC0_CFG1 = (0  | ADC_CFG1_ADIV(2)              //ʱ�ӷ�Ƶѡ��,��Ƶϵ��Ϊ 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK          //����ʱ�����ã�0Ϊ�̲���ʱ�䣬1 Ϊ������ʱ��
                     | ADC_CFG1_MODE(bit)            //��ȷ��ѡ��
                     | ADC_CFG1_ADICLK(0)            //0Ϊ����ʱ��,1Ϊ����ʱ��/2,2Ϊ����ʱ�ӣ�ALTCLK����3Ϊ �첽ʱ�ӣ�ADACK����
                 );


    ADC0_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //��������,0Ϊ����ת�����У�1Ϊ����ת������
                     | ADC_CFG2_ADLSTS(0)           //������ʱ��ѡ��ADCKΪ4+n������ѭ��������ѭ����0Ϊ20��1Ϊ12��2Ϊ6��3Ϊ2
                  );

    //д�� SC1A ����ת��
    ADC0_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // ת������ж�,0Ϊ��ֹ��1Ϊʹ��
                     //| ADC_SC1_ADCH( 0x0c )       //ADC0 ͨ��12
                     | ADC_SC1_ADCH( adc_ch )       //ADC0 ͨ��13
                  );
   }
   else
   {
     ADC1_CFG1 = (0   | ADC_CFG1_ADIV(2)              //ʱ�ӷ�Ƶѡ��,��Ƶϵ��Ϊ 2^n,2bit
                     | ADC_CFG1_ADLSMP_MASK           //����ʱ�����ã�0Ϊ�̲���ʱ�䣬1 Ϊ������ʱ��
                     | ADC_CFG1_MODE(bit)             //��ȷ��ѡ��
                     | ADC_CFG1_ADICLK(0)             //0Ϊ����ʱ��,1Ϊ����ʱ��/2,2Ϊ����ʱ�ӣ�ALTCLK����3Ϊ �첽ʱ�ӣ�ADACK����
                 );


    ADC1_CFG2  = (0  | ADC_CFG2_ADHSC_MASK          //��������,0Ϊ����ת�����У�1Ϊ����ת������
                     | ADC_CFG2_ADLSTS(0)           //������ʱ��ѡ��ADCKΪ4+n������ѭ��������ѭ����0Ϊ20��1Ϊ12��2Ϊ6��3Ϊ2
                  );

    //д�� SC1A ����ת��
    ADC1_SC1(0) = (0 | ADC_SC1_AIEN_MASK            // ת������ж�,0Ϊ��ֹ��1Ϊʹ��
                     //| ADC_SC1_ADCH( 0x0c )       //ADC0 ͨ��12
                     | ADC_SC1_ADCH( adc_ch )       //ADC0 ͨ��13
                  );
   }
}


/*------------------------------------------------------------------------------------------------------
����    ����ADC_Once
����    �ܡ�����ADCת�� ��ȡһ��ADCת��ֵ
����    ����adc_n ��  ģ����ADC0��ADC1
����    ����adc_ch��  ADCͨ�����
����    ����bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
���� �� ֵ����
��ʵ    ����ADC_Once(ADC0,ADC1_SE8,ADC_12bit) ͨ�����Ϊ ADC1_SE8�����ţ�����һ��ADCת����
��ע�����
--------------------------------------------------------------------------------------------------------*/
u16 ADC_Once(ADC_Type * adc_n, ADCn_Ch_e adc_ch, ADC_nbit bit)
{

    int result = 0;

    ADC_Start(adc_n,adc_ch,bit);      //����ADCת��
    if(adc_n==ADC0)
    {
        while ((ADC0_SC1(0) & ADC_SC1_COCO_MASK )!= ADC_SC1_COCO_MASK);   //ֻ֧�� Aͨ��
        result = ADC0_R(0);
        ADC0_SC1(0) &= ~ADC_SC1_COCO_MASK;
        return result;
    }
    else
    {
        while ((ADC1_SC1(0) & ADC_SC1_COCO_MASK )!= ADC_SC1_COCO_MASK);   //ֻ֧�� Aͨ��
        result = ADC1_R(0);
        ADC1_SC1(0) &= ~ADC_SC1_COCO_MASK;
        return result;
    }
}


/*------------------------------------------------------------------------------------------------------
����    ����ADC_Mid
����    �ܡ�ADC��������3��ȡ�м�ֵ
����    ����adc_n ��  ģ����ADC0��ADC1
����    ����adc_ch��  ADCͨ�����
����    ����bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
���� �� ֵ����ֵ
��ʵ    ����ADC_Mid(ADC0,ADC1_SE8,ADC_12bit) ͨ�����Ϊ ADC1_SE8�����ţ�����3��ADCת����ѡȡ�м�ֵ���ء�
��ע�����
--------------------------------------------------------------------------------------------------------*/
uint16_t ADC_Mid(ADC_Type * adc_n,ADCn_Ch_e adc_ch,ADC_nbit bit) //��ֵ�˲�
{
    uint16_t i,j,k,tmp;//1.ȡ3��A/Dת�����
    i = ADC_Once(adc_n,adc_ch,bit);
    j = ADC_Once(adc_n,adc_ch,bit);
    k = ADC_Once(adc_n,adc_ch,bit);

    //2.ȡ��ֵ
    if (i > j)
    {
        tmp = i; i = j; j = tmp;
    }
    if (k > j)
        tmp = j;
    else if(k > i)
        tmp = k;
    else
        tmp = i;

    return tmp;
}

/*------------------------------------------------------------------------------------------------------
����    ����ADC_Ave
����    �ܡ�ADC��������N�Σ�����ƽ��ֵ
����    ����adc_n ��  ģ����ADC0��ADC1
����    ����adc_ch��  ADCͨ�����
����    ����bit   ��  ����ѡ��ADC_8bit��ADC_12bit��ADC_10bit��ADC_16bit
����    ����N     :   ��ֵ�˲�������1~65535��
���� �� ֵ��ƽ��ֵ
��ʵ    ����ADC_Ave(ADC0,ADC1_SE8,ADC_12bit,100) ͨ�����Ϊ ADC1_SE8�����ţ�����100��ADCת���������ƽ��ֵ���ء�
��ע�����
--------------------------------------------------------------------------------------------------------*/
uint16_t ADC_Ave(ADC_Type * adc_n,ADCn_Ch_e adc_ch,ADC_nbit bit,uint16_t N) //��ֵ�˲�
{
    uint32_t tmp = 0;
    uint8_t  i;

    for(i = 0; i < N; i++)
    {
        tmp += ADC_Mid(adc_n,adc_ch,bit);
    }
    tmp = tmp / N;

    return (uint16_t)tmp;
}
