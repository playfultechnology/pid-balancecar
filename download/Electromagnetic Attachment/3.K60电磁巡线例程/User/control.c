#include "include.h"
#include "control.h"
#define T 0.1635f
#define L 0.143f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;
int count;
/*---------------------------------------------------------------
����    ����PIT0_Interrupt �����жϷ�����
����    �ܡ�PIT0���жϷ�����
����    ������
���� �� ֵ����
��ע�����ע������Ҫ����жϱ�־λ
----------------------------------------------------------------*/
void PIT1_IRQHandler()
{
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ
  if(Flag_Stop) Led_Flash(3,100);
  else          Led_Flash(2,100);
  if(delay_flag==1)
  {
    if(++delay_50==5)	 delay_50=0,delay_flag=0;//���������ṩ50ms�ľ�׼��ʱ
  }
  Encoder_Left=-FTM_ABGet(FTM2);//===��ȡ��������ֵ
  Encoder_Right=FTM_ABGet(FTM1);//===��ȡ��������ֵ

  if(Flag_Way==2)  Find_CCD_Zhongzhi();//��ȡCCD������ȡ����
  if(Flag_Way==3)  ELE_Sensor=(int)(Get_ELE_Bias());//��ȡ���Ѳ�ߴ�����ƫ����������
  Get_RC();
  Kinematic_Analysis(Velocity,Angle);    //С���˶�ѧ����
  Voltage=(int)(ADC_Ave(ADC1,ADC1_SE8,ADC_12bit,10)*33*11*10/4096);
  if(Turn_Off(Voltage)==0)                              		//===����������쳣
  {
    Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===�ٶȱջ����Ƽ�����A����PWM
    Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===�ٶȱջ����Ƽ�����B����PWM
    Xianfu_Pwm();                                                      //===PWM�޷�
    Set_Pwm(Motor_A,Motor_B,Servo);                                 	 //===��ֵ��PWM�Ĵ���
  }
  else
    Set_Pwm(0,0,SERVO_INIT);                                 		//===��ֵ��PWM�Ĵ���
  Key();    //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬

}

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=(int)(velocity*(1+T*tan(angle)/2/L));
		Target_B=(int)(velocity*(1-T*tan(angle)/2/L));      //���ֲ���
		Servo=(int)(SERVO_INIT+angle*K);                    //���ת��
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
  if(motor_a<0)
  {
    FTM_PwmDuty(MOTOA_FTM, MOTOA_CH1, 7200+motor_a);
    FTM_PwmDuty(MOTOA_FTM, MOTOA_CH2, 7200);
  }
  else
  {
    FTM_PwmDuty(MOTOA_FTM, MOTOA_CH1, motor_a);
    FTM_PwmDuty(MOTOA_FTM, MOTOA_CH2, 0);
  }

  if(motor_b<0)
  {
    FTM_PwmDuty(MOTOB_FTM, MOTOB_CH1, 7200+motor_b);
    FTM_PwmDuty(MOTOB_FTM, MOTOB_CH2, 7200);
  }
  else
  {
    FTM_PwmDuty(MOTOB_FTM, MOTOB_CH1, motor_b);
    FTM_PwmDuty(MOTOB_FTM, MOTOB_CH2, 0);
  }
  CMT_PwmDuty(servo);
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
		if(Servo<(SERVO_INIT-300))     Servo=SERVO_INIT-300;	  //����޷�
		if(Servo>(SERVO_INIT+300))     Servo=SERVO_INIT+300;		  //����޷�
}
/**************************************************************************
�������ܣ������޸�С������״̬
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=KEY_Scan(100);
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
  static u8 count;
  u8 temp;
  if(voltage<1110)
  {
    count++;
    if(count>100)count=0,Flag_Stop=1;
  }

  if(Flag_Stop==1) temp=1;   //��ص�ѹ����7.4V�رյ��
  else             temp=0;
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
		 Velocity=-LY/15;	  //�ٶȺ�ҡ�˵�������ء�
		 Angle=RX/200;
	}
		else	if(Flag_Way==2)//CCDѲ��
	{
		 Velocity=7;	   //CCDѲ��ģʽ���ٶ�
		 Bias=CCD_Zhongzhi-64;   //��ȡƫ��
		 Angle=Bias*0.010+(Bias-Last_Bias)*0.085; //PD����
		 Last_Bias=Bias;   //������һ�ε�ƫ��
	}
		else	if(Flag_Way==3)//���Ѳ��
	{
		 Velocity=7;	  //���Ѳ��ģʽ�µ��ٶ�
		 Bias=100-ELE_Sensor;  //��ȡƫ��
		 Angle=Bias*0.004+(Bias-Last_Bias)*0.02; //�ǳ���PID��ʽabs((int)(Bias))*Bias*0.0002+
		 Last_Bias=Bias;   //��һ�ε�ƫ��
	}
}






