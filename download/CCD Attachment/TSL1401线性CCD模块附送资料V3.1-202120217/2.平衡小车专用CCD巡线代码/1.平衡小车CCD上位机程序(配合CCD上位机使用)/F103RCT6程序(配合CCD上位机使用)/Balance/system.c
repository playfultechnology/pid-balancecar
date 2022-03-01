#include "system.h"
u16 ADV[128]={0};
u8 SciBuf[200];  //存储上传到上位机的信息
u8 CCD_Zhongzhi,CCD_Yuzhi;   //线性CCD  相关
void systemInit(void)
{ 
	//Delay function initialization
	//延时函数初始化
	delay_init();
	
	//Interrupt priority group setting
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试

	//Initialize the hardware interface connected to the LED lamp
	//初始化与LED灯连接的硬件接口
	
	delay_ms(500);					//=====延时等待系统稳定
	
	uart_init(115200);			//串口1初始化
	
	ccd_Init();							//初始化CCD 		
}
