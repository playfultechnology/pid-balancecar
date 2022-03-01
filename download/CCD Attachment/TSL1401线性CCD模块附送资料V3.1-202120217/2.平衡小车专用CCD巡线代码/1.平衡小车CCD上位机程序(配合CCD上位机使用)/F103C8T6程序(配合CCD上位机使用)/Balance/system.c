#include "system.h"
u16 ADV[128]={0};
u8 SciBuf[200];  //�洢�ϴ�����λ������Ϣ
u8 CCD_Zhongzhi,CCD_Yuzhi;   //����CCD  ���
void systemInit(void)
{ 
	//Delay function initialization
	//��ʱ������ʼ��
	delay_init();
	
	//Interrupt priority group setting
	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//Initialize the hardware interface connected to the LED lamp
	//��ʼ����LED�����ӵ�Ӳ���ӿ�
	LED_Init();                          
	
	delay_ms(500);					//=====��ʱ�ȴ�ϵͳ�ȶ�
	
	uart_init(115200);			//����1��ʼ��
	
	ccd_Init();							//��ʼ��CCD 		
}
