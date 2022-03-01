#include "include.h"
#include "ele.h"


#define Right_PIN       ADC1_SE4a//��E0
#define Middle_PIN      ADC1_SE5a//��E1
#define Left_PIN        ADC1_SE6a//��E2
#define ELE_ADC         ADC1     //ELE��������ADC����
/**************************************************************************
�������ܣ����Ѳ�ߴ��������ų�ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void ELE_Init(void)
{
  ADC_Init(ELE_ADC);//E0/E1/E2==ADC1_SE4a/ADC1_SE5a/ADC1_SE6a
}

/**************************************************************************
�������ܣ����Ѳ�ߴ�����ƫ���ȡ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
uint16 Get_ELE_Bias()
{
  int Sum,Error;//ע��Sum���������ͣ���Ҫ���

  Sensor_Left=ADC_Ave(ELE_ADC,Left_PIN,ADC_12bit,3);
  Sensor_Middle=ADC_Ave(ELE_ADC,Middle_PIN,ADC_12bit,3);
  Sensor_Right=ADC_Ave(ELE_ADC,Right_PIN,ADC_12bit,3);
  Sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //��һ������
  Error=Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);//��ȡƫ��
  return Error;

}