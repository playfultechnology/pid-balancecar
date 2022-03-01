#include "show.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
	if(Menu_MODE==1)//λ��ģʽ
	{
				OLED_ShowString(00,00,"POSITION MODE");                         //λ��ģʽ
				if(Flag_Stop==0)  OLED_ShowString(00,10,"MOTOR START");         //�������
				else              OLED_ShowString(00,10,"MOTOR CLOSE");         //����ر�
				//=============��3����ʾPosition_KP=======================//	
				OLED_ShowString(00,20,"KP:");
				OLED_ShowNumber(20,20,Position_KP,3,12);
				OLED_ShowString(37,20,"."),  
				OLED_ShowNumber(41,20,(int)(Position_KP*10)%10,1,12);		
				OLED_ShowString(75,20,"A:");	  
				OLED_ShowNumber(92,20,Amplitude_PKP,2,12);
				OLED_ShowString(104,20,"."),  
				OLED_ShowNumber(111,20,(int)(Amplitude_PKP*10)%10,1,12);		
				//=============��4����ʾPosition_KI=======================//	
				OLED_ShowString(00,30,"KI:");
				OLED_ShowNumber(20,30,Position_KI,3,12);
				OLED_ShowString(37,30,"."),  
				OLED_ShowNumber(41,30,(int)(Position_KI*10)%10,1,12);
				OLED_ShowString(75,30,"A:");	  
				OLED_ShowNumber(92,30,Amplitude_PKI,2,12);
				OLED_ShowString(104,30,"."),  
				OLED_ShowNumber(111,30,(int)(Amplitude_PKI*10)%10,1,12);		
			 //=============��5����ʾPosition_KD=======================//	
				OLED_ShowString(00,40,"KD:");
				OLED_ShowNumber(20,40,Position_KD,3,12);
				OLED_ShowString(37,40,"."),  
				OLED_ShowNumber(41,40,(int)(Position_KD*10)%10,1,12);
				OLED_ShowString(75,40,"A:");	  
				OLED_ShowNumber(92,40,Amplitude_PKD,2,12);
				OLED_ShowString(104,40,"."),  
				OLED_ShowNumber(111,40,(int)(Amplitude_PKD*10)%10,1,12);		
				//======���ǹ����˵� ѡ����Ҫ���ڵ�PID����											 
				 if(Menu_PID==1)      //����P����
			{			
			 OLED_ShowChar(60,20,'Y',12,1);
			 OLED_ShowChar(60,30,'N',12,1);
			 OLED_ShowChar(60,40,'N',12,1);
			}		
		     else if(Menu_PID==2)  //����I����
			{				
			 OLED_ShowChar(60,20,'N',12,1);
			 OLED_ShowChar(60,30,'Y',12,1);
			 OLED_ShowChar(60,40,'N',12,1);
			} 
		     else if(Menu_PID==3)  //����D����
			{				
			 OLED_ShowChar(60,20,'N',12,1);
			 OLED_ShowChar(60,30,'N',12,1);
			 OLED_ShowChar(60,40,'Y',12,1);
			}
			//=============��6����ʾ����λ�ú�Ŀ��λ��=======================//			
			OLED_ShowString(00,50,"T:");	 	OLED_ShowNumber(15,50,Target_Position,5,12) ;	 	  //Ŀ��λ�� Ĭ��10000
			OLED_ShowString(80,50,"P:");    OLED_ShowNumber(95,50,Encoder,5,12);              //����λ�� Ĭ��10000
	}
	   else
	   {
				OLED_ShowString(00,00,"VELOCITY MODE");                       //�ٶ�ģʽ
				if(Flag_Stop==0)  OLED_ShowString(00,10,"MOTOR START");         //�������
				else              OLED_ShowString(00,10,"MOTOR CLOSE");         //����ر�
			 //=============��3����ʾVelocity_KP=======================//	
				OLED_ShowString(00,20,"KP:");
				OLED_ShowNumber(20,20,Velocity_KP,3,12);
				OLED_ShowString(37,20,"."),  
				OLED_ShowNumber(41,20,(int)(Velocity_KP*10)%10,1,12);		
				OLED_ShowString(85,20,"A:");	  
				OLED_ShowNumber(98,20,Amplitude_VKP,1,12);
				OLED_ShowString(104,20,"."),  
				OLED_ShowNumber(111,20,(int)(Amplitude_VKP*10)%10,1,12);		
			//=============��4����ʾVelocity_KI=======================//	
				OLED_ShowString(00,30,"KI:");
				OLED_ShowNumber(20,30,Velocity_KI,3,12);
				OLED_ShowString(37,30,"."),  
				OLED_ShowNumber(41,30,(int)(Velocity_KI*10)%10,1,12);
				OLED_ShowString(85,30,"A:");	  
				OLED_ShowNumber(98,30,Amplitude_VKI,1,12);
				OLED_ShowString(104,30,"."),  
				OLED_ShowNumber(111,30,(int)(Amplitude_VKI*10)%10,1,12);		
			//======���ǹ����˵� ѡ����Ҫ���ڵ�PID����											
			 if(Menu_PID==1)    //����P����
				{			
				 OLED_ShowChar(60,20,'Y',12,1);
				 OLED_ShowChar(60,30,'N',12,1);
				}		
			 else if(Menu_PID==2) //����I����
				{				
				 OLED_ShowChar(60,20,'N',12,1);
				 OLED_ShowChar(60,30,'Y',12,1);
				}
	  	//=============��6����ʾ�����ٶȺ�Ŀ���ٶ�=======================//			
			OLED_ShowString(00,50,"T:");	 	OLED_ShowNumber(15,50,Target_Velocity,5,12) ;	 	//Ŀ���ٶ�
			OLED_ShowString(80,50,"P:");    OLED_ShowNumber(95,50,(u16)Encoder,5,12);            //�����ٶ�

	}
		OLED_Refresh_Gram();				//===ˢ��
}
void oled_show_once(void)
{
	OLED_ShowString(0,0,"Turn Right Wheel");	      //ת������̥
	OLED_ShowString(20,20,"Forward To");	 
	OLED_ShowString(30,30,"Position");	 
	OLED_ShowString(20,40,"Backward To");	 
	OLED_ShowString(30,50,"Velocity");	
	OLED_ShowString(00,10,"Encoder:");		OLED_ShowNumber(70,10,TIM4->CNT,5,12);    //��ʾ��Ϊ�������ı�����������
	OLED_Refresh_Gram();				//===ˢ��
}
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Encoder, 1 );      
if(Menu_MODE==1)		DataScope_Get_Channel_Data( Target_Position, 2 );      
else              	DataScope_Get_Channel_Data( Target_Velocity, 2 );  
//		DataScope_Get_Channel_Data( 0, 3 );              
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(2);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}

