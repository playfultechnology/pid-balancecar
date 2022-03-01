#include "stm32f4xx.h"
#include "sys.h"  
u16 ADV[128]={0};              
u8 CCD_Zhongzhi,CCD_Yuzhi;   //����CCD  ���
int main(void)
{
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
  OLED_Init();                    //=====OLED��ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	Adc_Init();                     //=====��ص�ѹ����adc��ʼ��
	TIM3_Int_Init(50-1,8400-1);	    //��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����50��Ϊ5ms     
  while(1){
		oled_show();            //===��ʾ����
	}
}
