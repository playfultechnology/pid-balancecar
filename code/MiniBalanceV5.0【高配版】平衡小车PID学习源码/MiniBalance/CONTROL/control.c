#include "control.h"	
#include "filter.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Velocity=1;
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
u32 Run_Count;
int Voltage_Temp,Voltage_Count,Voltage_All,Key_Count;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{   
		  EXTI->PR=1<<15;                                                      //����жϱ�־λ   
	  	Flag_Target=!Flag_Target;
		   if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
			 if(Run_Count++>300)     //3s֮���ִ�п��ƴ���
			 {
					 if( Flag_Target==0)  
					 {
							Get_Angle(1);  //===������̬	
							return 0;	  
					 }
						if(Menu_MODE==1)		//λ�ÿ���
					{
						Encoder=Read_Position(2);                      //===����λ����Ϣ
						Moto=Position_PID(Encoder,Target_Position);    //===λ��PID������
					}
					else if(++Flag_Velocity>2)                       //===�ٶȿ���20msһ�Σ�Ҳ��10ms
					{
						Flag_Velocity=1;
						Encoder=Read_Velocity(2);                      //===�����ٶ���Ϣ
						if(Encoder<0) Encoder=0;                       
						Moto=Incremental_PI(Encoder,Target_Velocity);      //===�ٶ�PI������
					}
						Encoder_Key=-Read_Velocity(4);   //��ȡ��·�����������ݿ���PID��������
					  if(++Key_Count>20) //200ms����һ��
						Key_Count=0,
						PID_Adjust(Encoder_Key);//����PID����
	  	 }
	    ///����4����Ҫ�ǵ�ص����ɼ�
			Voltage_Temp=Get_battery_volt();		                                //=====��ȡ��ص�ѹ		
			Voltage_Count++;                                                    //=====ƽ��ֵ������
			Voltage_All+=Voltage_Temp;                                          //=====��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====��ƽ��ֵ		                                                                 
	  	Get_Angle(1);                                                       //===������̬	
  		Led_Flash(100);                                                     //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
			Key();                                                              //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬

   		Xianfu_Pwm();                                                       //===PWM�޷�
      if(Turn_Off(Angle_Balance,Voltage)==0)                              //===����������쳣
 			Set_Pwm(0,Moto);                                                    //===��ֵ��PWM�Ĵ���  
	}       	
	 return 0;	  
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
//    	if(moto1<0)			AIN2=0,			AIN1=1;
//			else 	          AIN2=1,			AIN1=0;
//			PWMA=myabs(moto1);
	    AIN1=0;  
	    AIN2=0;
	    PWMA=0;
		  if(moto2<0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;	
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	int tmp,tmp2,Position_Amplitude=390,Velocity_Amplitude=10; 
	static u8 mode=0;
	tmp=click_N_Double(100); 
	 if(Menu_MODE==1)  //�ı�����λ�� 
	 {
		 	if(tmp==1)Target_Position+=Position_Amplitude;
			if(tmp==2)Target_Position-=Position_Amplitude;
	 }
	 else             //�ı������ٶ�
	 {
			if(tmp==1)Target_Velocity+=Velocity_Amplitude;
			if(tmp==2)Target_Velocity-=Velocity_Amplitude;
	 }
	 if(Target_Velocity>70)Target_Velocity=70; //�ٶ����ֵ�޷�
	 if(Target_Velocity<0)Target_Velocity=0;   //�ٶ���Сֵ�޷�
	 if(Menu_MODE==1)  mode=3;
	 else mode=2;
   tmp2=Long_Press();        //�����û�������ѡ����Ҫ�޸ĵĲ���
   if(tmp2==1) 
	 {
		 Menu_PID++;
	  if(Menu_PID>mode)  Menu_PID=1;
	 }		
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
				if(angle<-40||angle>40||voltage<1110)//��ص�ѹ����11.1V�رյ��
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                           
			AIN1=0;                                            
			AIN2=0;
			BIN1=0;
			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
			{	
					Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
					Angle_Balance=-Roll;             //===����ƽ�����		
			}			
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000) Integral_bias=100000;
	 if(Integral_bias<-100000) Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias/10+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
void PID_Adjust(int encoder)
{
if(Menu_MODE==1)
	{	
			if(encoder<-3)	//PID-
			{
				      if(Menu_PID==1)  Position_KP-=Amplitude_PKP;
				else	if(Menu_PID==2)  Position_KI-=Amplitude_PKI;
				else	if(Menu_PID==3)  Position_KD-=Amplitude_PKD;
			}		
			 if(encoder>3)		//PID+ 
			{
							if(Menu_PID==1)  Position_KP+=Amplitude_PKP;
				else	if(Menu_PID==2)  Position_KI+=Amplitude_PKI;
				else	if(Menu_PID==3)  Position_KD+=Amplitude_PKD;
			}		
  }
	else
	{	
			if(encoder<-3)		//PID-
			{
				      if(Menu_PID==1)  Velocity_KP-=Amplitude_VKP;
				else	if(Menu_PID==2)  Velocity_KI-=Amplitude_VKI;
			}		
			if(encoder>3)	//PID+ 
			{
							if(Menu_PID==1)  Velocity_KP+=Amplitude_VKP;
				else	if(Menu_PID==2)  Velocity_KI+=Amplitude_VKI;
			}		
  }
}
