
#include "include.h"

#define T 0.156f
#define L 0.1445f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;



/*---------------------------------------------------------------
����    ����PIT0_Interrupt �����жϷ�����
����    �ܡ�PIT0���жϷ�����
����    ������
���� �� ֵ����
��ע�����ע������Ҫ����жϱ�־λ
----------------------------------------------------------------*/
void PIT0_IRQHandler()
{
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ

    
    if(delay_flag==1)
			 {
				 if(++delay_50==5)	 delay_50=0,delay_flag=0;                      //���������ṩ50ms�ľ�׼��ʱ
			 }
						Encoder_Left=Read_Encoder(2);                                       //===��ȡ��������ֵ							 //Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
						Encoder_Right=-Read_Encoder(3);                                      //===��ȡ��������ֵ
					 if(Flag_Way==2)
					 {	 
								RD_TSL();                           		 //===��ȡ����CCD���� 
						 	  Find_CCD_Zhongzhi();			          		 //===��ȡ���� 
					 }
					 if(Flag_Way==3)		
					 {
					 			Sensor_Left=Get_Adc(3);                //�ɼ���ߵ�е�����
								Sensor_Right=Get_Adc(8);               //�ɼ��ұߵ�е�����
								Sensor_Middle=Get_Adc(2);              //�ɼ��м��е�����
					  	  sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //��һ������
								Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //��ƫ��
					 }						 
						Get_RC();
						Kinematic_Analysis(Velocity,Angle);     															//С���˶�ѧ����   
						if(Turn_Off(Voltage)==0)                              							 //===����������쳣
						{
							Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===�ٶȱջ����Ƽ�����A����PWM
							Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===�ٶȱջ����Ƽ�����B����PWM 
							Xianfu_Pwm();                                                      //===PWM�޷�
							Set_Pwm(Motor_A,Motor_B,Servo);                                 	 //===��ֵ��PWM�Ĵ���  
						}
						else
						Set_Pwm(0,0,SERVO_INIT);                                 						 //===��ֵ��PWM�Ĵ���  	
						Voltage_Temp=Get_battery_volt();		                                 //=====��ȡ��ص�ѹ		
						Voltage_Count++;                                                     //=====ƽ��ֵ������
						Voltage_All+=Voltage_Temp;                                           //=====��β����ۻ�
						if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====��ƽ��ֵ		                                   
			  
			 if(Flag_Show==0)				Led_Flash(100);
			 else if(Flag_Show==1)	Led_Flash(0);  //led��˸
			 Key();    //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬
    
}

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=velocity*(1+T*tan(angle)/2/L); 
		Target_B=velocity*(1-T*tan(angle)/2/L);      //���ֲ���
		Servo=SERVO_INIT+angle*K;                    //���ת��   
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	    if(Flag_Way>=2)//Ѳ��ģʽ�£�ֻ��������ת
			{
			  if(motor_a<0)motor_a=0;
				if(motor_b<0)motor_b=0;
			}
    	if(motor_a<0)			PWMA1=7200,PWMA2=7200+motor_a;
			else 	            PWMA2=7200,PWMA1=7200-motor_a;
		
		  if(motor_b<0)			PWMB1=7200,PWMB2=7200+motor_b;
			else 	            PWMB2=7200,PWMB1=7200-motor_b;
     SERVO=servo;	
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(Motor_A<-Amplitude) Motor_A=-Amplitude;	
		if(Motor_A>Amplitude)  Motor_A=Amplitude;	
	  if(Motor_B<-Amplitude) Motor_B=-Amplitude;	
		if(Motor_B>Amplitude)  Motor_B=Amplitude;		
		if(Servo<(SERVO_INIT-500))     Servo=SERVO_INIT-500;	  //����޷�
		if(Servo>(SERVO_INIT+500))     Servo=SERVO_INIT+500;		  //����޷�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click(); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//��������С������ͣ
	//if(tmp==2)Flag_Show=!Flag_Show;//˫������С������ʾ״̬
	tmp2=Long_Press();          
  if(tmp2==1)Flag_Show=!Flag_Show;//����С������ʾ״̬
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<740||Flag_Stop==1)//��ص�ѹ����7.4V�رյ��
			{	                                                
      temp=1;                                            
      }
			else
      temp=0;
      return temp;			
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
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	int Yuzhi=2;
	static float Bias,Last_Bias;
  float LY,RX;
	if(Flag_Way==0)//��������
	{
		if(Flag_Direction==0) Velocity=0,Angle=0;   //ֹͣ
		else if(Flag_Direction==1) Velocity=Bluetooth_Velocity,Angle=0;  //ǰ��
		else if(Flag_Direction==2) Velocity=Bluetooth_Velocity,Angle=PI/5;  //��ǰ
		else if(Flag_Direction==3) Velocity=0,Angle=0;   //�������
		else if(Flag_Direction==4) Velocity=-Bluetooth_Velocity,Angle=PI/5;  // �Һ�
		else if(Flag_Direction==5) Velocity=-Bluetooth_Velocity,Angle=0;    //����
		else if(Flag_Direction==6) Velocity=-Bluetooth_Velocity,Angle=-PI/5;  //���
		else if(Flag_Direction==7) Velocity=0,Angle=0;                       //�������
		else if(Flag_Direction==8) Velocity=Bluetooth_Velocity,Angle=-PI/5;  //��ǰ
	}
	else	if(Flag_Way==1)//PS2����
	{
  	LY=PS2_LY-128;     //����ƫ��
		RX=PS2_RX-128;
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0;   //С�Ƕ���Ϊ���� ��ֹ���������쳣
	  if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
		 Velocity=-LY/2;	  //�ٶȺ�ҡ�˵�������ء�
		 Angle=RX/200; 
	}
		else	if(Flag_Way==2)//CCDѲ��
	{
		 Velocity=45;	   //CCDѲ��ģʽ���ٶ�
		 Bias=CCD_Zhongzhi-64;   //��ȡƫ��
		 Angle=Bias*0.010+(Bias-Last_Bias)*0.085; //PD����
		 Last_Bias=Bias;   //������һ�ε�ƫ��
	}
		else	if(Flag_Way==3)//���Ѳ��
	{
		 Velocity=45;	  //���Ѳ��ģʽ�µ��ٶ�
		 Bias=100-Sensor;  //��ȡƫ��
		 Angle=myabs(Bias)*Bias*0.0002+Bias*0.001+(Bias-Last_Bias)*0.05; //
		 Last_Bias=Bias;   //��һ�ε�ƫ��
	}
}
/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
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
	 for(i = 5;i<118; i++)   //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ�������
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


