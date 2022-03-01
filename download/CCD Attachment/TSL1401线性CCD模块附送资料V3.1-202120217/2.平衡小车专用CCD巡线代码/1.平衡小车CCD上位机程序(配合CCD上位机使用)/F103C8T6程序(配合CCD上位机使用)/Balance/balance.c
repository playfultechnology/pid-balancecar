#include "balance.h"

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	 //阈值说明：CCD采集回来的128个数据，每个数据单独与阈值进行比较，比阈值大为白色，比阈值小为黑色
	 //动态阈值算法，读取每次采集数据的最大和最小值的平均数作为阈值 
	 value1_max=ADV[0];  
   for(i=5;i<123;i++)   //两边各去掉5个点
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	  value1_min=ADV[0];  //最小值
    for(i=5;i<123;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿，连续三个白像素后连续三个黑像素判断左边跳变沿
	 {
		 if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		 {	
			 Left=i;
			 break;	
		 }
	 }
	 for(j = 118;j>5; j--)//寻找右边跳变沿，连续三个黑像素后连续三个白像素判断左边跳变沿
   {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		 {	
		   Right=j;
		   break;	
		 }
   }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}
