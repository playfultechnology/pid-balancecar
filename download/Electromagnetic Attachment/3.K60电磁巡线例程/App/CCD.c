#include "include.h"
#include "ccd.h"

#define TSL_SI    PTE2_OUT   //==CCD������SI����
#define TSL_CLK   PTE1_OUT   //==CCD������CLK����
#define CCD_AO    ADC1_SE4a  //==CCD��������������E0����
#define CCD_ADC   ADC1       //==CCD�����ĸ�ADC����
uint16 ADV[128]={0};         //==CCD��ȡ���ݱ���
/**************************************************************************
�������ܣ�CCD���������ų�ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void CCD_Init(void)
{
  GPIO_PinInit(PTE2, GPO, 0);//PTE1,����������ߵ�ƽ
  GPIO_PinInit(PTE1, GPO, 1);//PTE2,����������ߵ�ƽ
  ADC_Init(CCD_ADC);//E0==ADC1_SE4a
}
/**************************************************************************
�������ܣ�������ʱ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void CCD_delay(void)
{
   int ii;
   for(ii=0;ii<10;ii++);
}
/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
 void Read_TSL(void)
 {
   u8 i=0,tslp=0;
   TSL_CLK=1;
   TSL_SI=0;
   CCD_delay();

   TSL_SI=1;
   TSL_CLK=0;
   CCD_delay();

   TSL_CLK=1;
   TSL_SI=0;
   CCD_delay();
   for(i=0;i<128;i++)
   {
     TSL_CLK=0;
     CCD_delay();  //�����ع�ʱ��
     CCD_delay();  //�����ع�ʱ��
     ADV[tslp]=(ADC_Once(CCD_ADC,CCD_AO,ADC_12bit))>>4;//���ζ�ȡ128������
     ++tslp;
     TSL_CLK=1;
     CCD_delay();
   }
 }

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{
  static u16 i,j,Left,Right;
//  static u16 Last_CCD_Zhongzhi;
  static u16 value1_max,value1_min;
  Read_TSL();
  value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
  for(i=5;i<123;i++)   //���߸�ȥ��5���㣬��Ե��׼ȷ
  {
    if(value1_max<=ADV[i])//�����ֵ
      value1_max=ADV[i];//�����ֵ
  }
  value1_min=ADV[0];
  for(i=5;i<123;i++)//���߸�ȥ��5���㣬��Ե��׼ȷ
  {
    if(value1_min>=ADV[i])//����Сֵ
      value1_min=ADV[i];//����Сֵ
  }

  CCD_Yuzhi=(value1_max+value1_min)/2;//������ֵ
  for(i = 5;i<118; i++)   //Ѱ�����������
  {
    if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)//����ȷ��5��������
    {
      Left=i;//��¼���������������
      break;
    }
  }
  for(j = 118;j>5; j--)//Ѱ���ұ�������
  {
    if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
    {
      Right=j;//��¼�ұ�������������
      break;
    }
  }
  CCD_Zhongzhi=(Right+Left)/2;//��������λ��
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}