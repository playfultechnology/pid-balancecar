#include "include.h"
#include "ele.h"


#define Right_PIN       ADC1_SE4a//右E0
#define Middle_PIN      ADC1_SE5a//中E1
#define Left_PIN        ADC1_SE6a//左E2
#define ELE_ADC         ADC1     //ELE引脚所在ADC外设
/**************************************************************************
函数功能：电磁巡线传感器引脚初始化
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void ELE_Init(void)
{
  ADC_Init(ELE_ADC);//E0/E1/E2==ADC1_SE4a/ADC1_SE5a/ADC1_SE6a
}

/**************************************************************************
函数功能：电磁巡线传感器偏差获取
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
uint16 Get_ELE_Bias()
{
  int Sum,Error;//注意Sum的数据类型，不要溢出

  Sensor_Left=ADC_Ave(ELE_ADC,Left_PIN,ADC_12bit,3);
  Sensor_Middle=ADC_Ave(ELE_ADC,Middle_PIN,ADC_12bit,3);
  Sensor_Right=ADC_Ave(ELE_ADC,Right_PIN,ADC_12bit,3);
  Sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //归一化处理
  Error=Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);//获取偏差
  return Error;

}