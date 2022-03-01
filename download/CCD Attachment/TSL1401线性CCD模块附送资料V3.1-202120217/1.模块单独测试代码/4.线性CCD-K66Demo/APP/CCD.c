#include "include.h"
#include "ccd.h"

#define TSL_SI    PTE2_OUT   //==CCD传感器SI引脚
#define TSL_CLK   PTE1_OUT   //==CCD传感器CLK引脚
#define CCD_AO    ADC1_SE4a  //==CCD传感器数据引脚E0引脚
#define CCD_ADC   ADC1       //==CCD所在哪个ADC外设
uint16 ADV[128]={0};         //==CCD读取数据保存
/**************************************************************************
函数功能：CCD传感器引脚初始化
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void CCD_Init(void)
{
  GPIO_PinInit(PTE2, GPO, 0);//PTE1,推挽输出，高电平
  GPIO_PinInit(PTE1, GPO, 1);//PTE2,推挽输出，高电平
  ADC_Init(CCD_ADC);//E0==ADC1_SE4a
}
/**************************************************************************
函数功能：计数延时
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void CCD_delay(void)
{
   int ii;
   for(ii=0;ii<10;ii++);
}
/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
作    者：平衡小车之家
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
     CCD_delay();  //调节曝光时间
     CCD_delay();  //调节曝光时间
     ADV[tslp]=(ADC_Once(CCD_ADC,CCD_AO,ADC_12bit))>>4;//依次读取128个数据
     ++tslp;
     TSL_CLK=1;
     CCD_delay();
   }
 }

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{
  static u16 i,j,Left,Right;
//  static u16 Last_CCD_Zhongzhi;
  static u16 value1_max,value1_min;
  Read_TSL();
  value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
  for(i=5;i<123;i++)   //两边各去掉5个点，边缘不准确
  {
    if(value1_max<=ADV[i])//找最大值
      value1_max=ADV[i];//找最大值
  }
  value1_min=ADV[0];
  for(i=5;i<123;i++)//两边各去掉5个点，边缘不准确
  {
    if(value1_min>=ADV[i])//找最小值
      value1_min=ADV[i];//找最小值
  }

  CCD_Yuzhi=(value1_max+value1_min)/2;//计算阈值
  for(i = 5;i<118; i++)   //寻找左边跳变沿
  {
    if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)//连续确认5个像数点
    {
      Left=i;//记录左边沿跳变像数点
      break;
    }
  }
  for(j = 118;j>5; j--)//寻找右边跳变沿
  {
    if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
    {
      Right=j;//记录右边沿跳变像数点
      break;
    }
  }
  CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}