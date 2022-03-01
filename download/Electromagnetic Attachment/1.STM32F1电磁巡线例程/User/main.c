#include "stm32f10x.h"
#include "sys.h"


int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//���Ѳ�����



int main(void)
  {
		delay_init();	    	        //=====��ʱ������ʼ��	
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
		LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
		MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
		OLED_Init();					//=====OLED��ʼ��
		uart_init(9600);				//=====����1��ʼ��
	  
		ele_Init();  //=====��Ŵ�������ʼ��	

		
	while(1)
	{
		int  sum;
		
	
		Sensor_Left=Get_Adc(11);                //�ɼ���ߵ�е�����
		Sensor_Right=Get_Adc(13);               //�ɼ��ұߵ�е�����
		Sensor_Middle=Get_Adc(12);              //�ɼ��м��е�����
		sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //��һ������
		Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //��ƫ��
		
		
		oled_show();
		
		printf("      Left: %d\r",Sensor_Left);
		printf("      Middle: %d\r",Sensor_Middle);
		printf("      Right: %d\r",Sensor_Right);
		printf("      Sensor: %d\r\n",Sensor);
		
		delay_ms(50);
		
	  } 
}

