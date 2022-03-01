#include "control.h"	
#include "filter.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All; 											 //��ѹ������ر���
float Gyro_K=0.004;     				  											//�����Ǳ���ϵ��
int j,sum;
#define a_PARAMETER          (0.095f)               
#define b_PARAMETER          (0.086f)         
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	Target_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Target_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
	
	Rate_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Rate_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Rate_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	Rate_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ  		
		   Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
			{
				if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ�����봮�ڿ���ģʽ
				Read_DMP();                                                           //===������̬	
				if(Flag_Way==2)
					 {	 
						RD_TSL();  //===��ȡ����CCD���� 
						 Find_CCD_Zhongzhi(); //===��ȡ���� 
					 }
				if(Flag_Way==3)		
					 {
					 	Sensor_Left=Get_Adc(11);                //�ɼ���ߵ�е�����
						Sensor_Right=Get_Adc(13);               //�ɼ��ұߵ�е�����
						Sensor_Middle=Get_Adc(12);              //�ɼ��м��е�����
					  	sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //��һ������
						Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //��ƫ��
					 }	
				Key();//ɨ�谴���仯	
			return 0;	                                               
			}                                                                      //===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
			Encoder_A=Read_Encoder(2);                                         		 //===��ȡ��������ֵ
			Position_A+=Encoder_A;                                                 //===���ֵõ��ٶ�   
			Encoder_B=Read_Encoder(3);                                         		 //===��ȡ��������ֵ
			Position_B+=Encoder_B;                                                 //===���ֵõ��ٶ�   
			Encoder_C=-Read_Encoder(4);                                         		//===��ȡ��������ֵ
			Position_C+=Encoder_C;                                                 //===���ֵõ��ٶ�   
			Encoder_D=-Read_Encoder(5);                                         		//===��ȡ��������ֵ
			Position_D+=Encoder_D;                                                 //===���ֵõ��ٶ�   
			Read_DMP();                                                            //===������̬	
			Led_Flash(100);                                                        //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
			Voltage_All+=Get_battery_volt();                                       //��β����ۻ�
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	 
//			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;                   //������������ٷ�������
		 if(Turn_Off(Voltage)==0)         										 //===�����ص�ѹ�������쳣
		 { 			 
		  if(Run_Flag==0)//�ٶ�ģʽ
			{		
				Get_RC(Run_Flag);
				Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===�ٶȱջ����Ƽ�����A����PWM
				Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===�ٶȱջ����Ƽ�����B����PWM
				Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===�ٶȱջ����Ƽ�����C����PWM
				Motor_D=Incremental_PI_D(Encoder_D,Target_D);                         //===�ٶȱջ����Ƽ�����C����PWM
			}
			 else//λ��ģʽ
			{
				 Get_RC(Run_Flag);
					Motor_A=Position_PID_A(Position_A,Target_A)>>8;//λ�ñջ����ƣ�������A�ٶ��ڻ���������
					Motor_B=Position_PID_B(Position_B,Target_B)>>8;//λ�ñջ����ƣ�������B�ٶ��ڻ���������
					Motor_C=Position_PID_C(Position_C,Target_C)>>8;//λ�ñջ����ƣ�������C�ٶ��ڻ���������
				 	Motor_D=Position_PID_D(Position_D,Target_D)>>8;//λ�ñջ����ƣ�������C�ٶ��ڻ���������
				
			   if(rxbuf[0]!=2)  Count_Velocity();   //���ǵ���λ�ÿ��ƹ��̵��ٶȴ�С
					else 	
					Xianfu_Velocity(RC_Velocity,RC_Velocity,RC_Velocity,RC_Velocity); 
					Motor_A=Incremental_PI_A(Encoder_A,-Motor_A);         //===�ٶȱջ����Ƽ�����A����PWM
					Motor_B=Incremental_PI_B(Encoder_B,-Motor_B);         //===�ٶȱջ����Ƽ�����B����PWM
					Motor_C=Incremental_PI_C(Encoder_C,-Motor_C);         //===�ٶȱջ����Ƽ�����C����PWM
					Motor_D=Incremental_PI_D(Encoder_D,-Motor_D);         //===�ٶȱջ����Ƽ�����C����PWM
			}	 
		 Xianfu_Pwm(6900);                     //===PWM�޷�
		 Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===��ֵ��PWM�Ĵ���  
		 }
 }
	 return 0;	 
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	    int siqu=0;
    	if(motor_a>0)			PWMA=motor_a+siqu, INA=0;
			else  	          PWMA=7200+motor_a-siqu,INA=1;
		
		  if(motor_b>0)			PWMB=7200-motor_b-siqu,INB=1;
			else 	            PWMB=-motor_b+siqu,INB=0;
	
	    if(motor_c>0)			PWMC=7200-motor_c-siqu,INC=1;
			else 	            PWMC=-motor_c+siqu,INC=0;
	
	   if(motor_d>0)			PWMD=motor_d+siqu,IND=0;
			else 	            PWMD=7200+motor_d-siqu,IND=1;
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;		
	  if(Motor_D<-amplitude) Motor_D=-amplitude;	
	  if(Motor_D>amplitude)  Motor_D=amplitude;		
}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
	  if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//λ�ÿ���ģʽ�У�C����������ٶ�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(50); 
	if(tmp==2)Flag_Show=!Flag_Show;//˫��������ʾģʽ  
	if(tmp==1)Flag_Stop=!Flag_Stop;//�������Ƶ����ͣ	
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110||Flag_Stop==1)//��ص�ѹ���͹رյ��
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;
      PWMC=0;
	  PWMD=0;
      INA=0;
      INB=0;
      INC=0;		
	  IND=0;								
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
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
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
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
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_D (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(u8 mode)
{
	int flag_1, flag_2,flag_3,flag_4;
	int LX,LY,RX;	//PS2�ֱ����Ʊ���
	int Yuzhi=20;	//PS2���Ʒ�����ֵ
	float step=0.5; //�����ٶȿ��Ʋ���ֵ��
	static float Bias,Last_Bias;  //ƫ�����ʷֵ
	u8 Flag_Move=1;
	if(mode==0)//�ٶ�ģʽ
	{
		if(Flag_Way==0)//APPģʽ��
		{
			if(CAN_ON_Flag==0&&Usart_ON_Flag==0) //CAN/USARTδʹ�ܣ���������ָ��
			{
				switch(Flag_Direction)   //�������
				 {
				 case 1:  Move_X=0;           Move_Y+=step;  Flag_Move=1;               break;
				 case 2:  Move_X+=step;       Move_Y+=step;  Flag_Move=1;               break;
				 case 3:  Move_X+=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 4:  Move_X+=step;       Move_Y-=step;  Flag_Move=1;               break;
				 case 5:  Move_X=0;           Move_Y-=step;  Flag_Move=1;               break;
				 case 6:  Move_X-=step;       Move_Y-=step;  Flag_Move=1;               break;
				 case 7:  Move_X-=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 8:  Move_X-=step;       Move_Y+=step;  Flag_Move=1;               break; 
				 default: Flag_Move=0;   		Move_X=Move_X/1.05;Move_Y=Move_Y/1.05;      break;	 
				}
				 
				if(Flag_Move==0)		//����޷������ָ��	 �����ת�����״̬
				{	
				if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //������   
				else if(Flag_Right==1)  Move_Z+=step,Gyro_K=0;    //������		
				else 		            Move_Z=0,Gyro_K=0.004;    //ֹͣ
				}	
				if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			}
			else//CAN���ߴ��ڿ���
			{
				
				if(rxbuf[0]==1)
				{
					if((rxbuf[7]&0x04)==0)flag_1=1;  else flag_1=-1;  //�������λ
				    if((rxbuf[7]&0x02)==0)flag_2=1;  else flag_2=-1;  //�������λ
					if((rxbuf[7]&0x01)==0)flag_3=1;  else flag_3=-1;  //�������λ
					Move_X=flag_1*(rxbuf[1]*256+rxbuf[2]);
					Move_Y=flag_2*(rxbuf[3]*256+rxbuf[4]);	
					Move_Z=flag_3*(rxbuf[5]*256+rxbuf[6]);	
					Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�����˶�ѧ����
					Gyro_K=0;    
				}
				if(rxbuf[0]==2)
				{
					if((rxbuf[7]&0x08)==0)flag_1=1;  else flag_1=-1;  //�������λ
					if((rxbuf[7]&0x04)==0)flag_2=1;  else flag_2=-1;  //�������λ
					if((rxbuf[7]&0x02)==0)flag_3=1;  else flag_3=-1;  //�������λ
					if((rxbuf[7]&0x01)==0)flag_4=1;  else flag_4=-1;  //�������λ
					Target_A=flag_1*rxbuf[1];
					Target_B=flag_2*rxbuf[2];
					Target_C=flag_3*rxbuf[3];
					Target_D=flag_4*rxbuf[4];
				 }
			}
		}
		else if(Flag_Way==1)//PS2����
		{
			LX=PS2_LX-128; //��ȡƫ��
			LY=PS2_LY-128; //��ȡƫ��
			RX=PS2_RX-128; //��ȡƫ��
			if(LX>-Yuzhi&&LX<Yuzhi)LX=0; //����С�Ƕȵ�����
			if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //����С�Ƕȵ�����
			if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //����С�Ƕȵ�����
			 Move_X=LX*RC_Velocity/200;
			 Move_Y=-LY*RC_Velocity/200;	
			 Move_Z=-RX*RC_Velocity/40;		
		}
		else if(Flag_Way==2)//CCDѲ��
		{
			 RC_Velocity=45;//Ѳ���ٶ�
	   	 Move_Y=RC_Velocity;
			 Bias=CCD_Zhongzhi-64;   //��ȡƫ��
			 Move_Z=-Bias*4-(Bias-Last_Bias)*25; //PD����
			 Last_Bias=Bias;   //������һ�ε�ƫ��
		}
		else if(Flag_Way==3)//���Ѳ��
		{
			 RC_Velocity=45;//Ѳ���ٶ�
			 Move_Y=RC_Velocity;
			 Bias=100-Sensor;  //��ȡƫ��
			 Move_Z=abs(Bias)*Bias*0.02+Bias*0.1+(Bias-Last_Bias)*5; //
			 Last_Bias=Bias;   //��һ�ε�ƫ��
		}
	}
	else if(mode==1)//λ��ģʽ
	{
		if(CAN_ON_Flag==0&&Usart_ON_Flag==0) //CAN/USARTδʹ�ܣ���������ָ��
		{
			if(Turn_Flag==0) 	Flag_Direction=click_RC();
			switch(Flag_Direction)   //�������
				 {
				 case 1:  Move_Y+=RC_Position; Flag_Change=1;break;
				 case 2:  Move_X+=RC_Position; Flag_Change=2; Move_Y+=RC_Position; break;
				 case 3:  Move_X+=RC_Position; Flag_Change=3;break;
				 case 4:  Move_X+=RC_Position; Flag_Change=4;Move_Y-=RC_Position;break;
				 case 5:  Move_Y-=RC_Position; Flag_Change=5;break;
				 case 6:  Move_X-=RC_Position; Flag_Change=6;Move_Y-=RC_Position; break;
				 case 7:  Move_X-=RC_Position; Flag_Change=7; break;
				 case 8:  Move_X-=RC_Position; Flag_Change=8;Move_Y+=RC_Position;break;			 
				 case 9:  Move_Z-=RC_Position; Flag_Change=9; break;
				 case 10: Move_Z+=RC_Position; Flag_Change=10;break;			 
				 default: break;	 
			 }
		 }
		else //����/CAN����
		{
			if(rxbuf[0]==1)
			{
			 if((rxbuf[7]&0x04)==0)flag_1=1;  else flag_1=-1;  //�������λ
			 if((rxbuf[7]&0x02)==0)flag_2=1;  else flag_2=-1;  //�������λ
			 if((rxbuf[7]&0x01)==0)flag_3=1;  else flag_3=-1;  //�������λ
			 Move_X=flag_1*(rxbuf[1]*256+rxbuf[2]);
			 Move_Y=flag_2*(rxbuf[3]*256+rxbuf[4]);	
			 Move_Z=flag_3*(rxbuf[5]*256+rxbuf[6]);	
			 Gyro_K=0;    
		 }
		}
	 }
	if(IR1==0||IR2==0) Move_X=0,Move_Y=0,Move_Z=0;//����м�⵽�ϰ���ֹͣ
		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}
/**************************************************************************
�������ܣ�ÿ�����λ�ÿ��ƹ����ٶȼ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Count_Velocity(void)
{
	static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
	double Bias_X,Bias_Y,Bias_Z;
	Bias_X=(Move_X-Last_Target_X);  //��X��λ����
	Bias_Y=(Move_Y-Last_Target_Y);	//��Y��λ����
	Bias_Z=(Move_Z-Last_Target_Z);	//��Z��λ����
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

	Xianfu_Velocity(RC_Velocity*myabs(Rate_A)/Divider,RC_Velocity*myabs(Rate_B)/Divider,RC_Velocity*myabs(Rate_C)/Divider,RC_Velocity*myabs(Rate_D)/Divider); 
	Last_Target_X=Move_X;   //����X����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Y=Move_Y;   //����Y����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Z=Move_Z;   //����Z����һ�ε�λ����Ϣ�����ڵ���
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
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}






