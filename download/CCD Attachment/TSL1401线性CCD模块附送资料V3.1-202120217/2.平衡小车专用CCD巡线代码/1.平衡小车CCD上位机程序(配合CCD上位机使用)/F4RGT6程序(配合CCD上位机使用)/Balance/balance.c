#include "balance.h"

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	 //��ֵ˵����CCD�ɼ�������128�����ݣ�ÿ�����ݵ�������ֵ���бȽϣ�����ֵ��Ϊ��ɫ������ֵСΪ��ɫ
	 //��̬��ֵ�㷨����ȡÿ�βɼ����ݵ�������Сֵ��ƽ������Ϊ��ֵ 
	 value1_max=ADV[0];  
   for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	  value1_min=ADV[0];  //��Сֵ
    for(i=5;i<123;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
		 
	 for(i = 5;i<118; i++)   //Ѱ����������أ��������������غ����������������ж����������
	 {
		 if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		 {	
			 Left=i;
			 break;	
		 }
	 }
	 for(j = 118;j>5; j--)//Ѱ���ұ������أ��������������غ����������������ж����������
   {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		 {	
		   Right=j;
		   break;	
		 }
   }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}
