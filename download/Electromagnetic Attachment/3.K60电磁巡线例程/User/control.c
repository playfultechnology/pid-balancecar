#include "include.h"
#include "control.h"
#define T 0.1635f
#define L 0.143f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;
int count;
/*---------------------------------------------------------------
【函    数】PIT0_Interrupt 核心中断服务函数
【功    能】PIT0的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void PIT1_IRQHandler()
{
  PIT_Flag_Clear(PIT1);       //清中断标志位
  if(Flag_Stop) Led_Flash(3,100);
  else          Led_Flash(2,100);
  if(delay_flag==1)
  {
    if(++delay_50==5)	 delay_50=0,delay_flag=0;//给主函数提供50ms的精准延时
  }
  Encoder_Left=-FTM_ABGet(FTM2);//===读取编码器的值
  Encoder_Right=FTM_ABGet(FTM1);//===读取编码器的值

  if(Flag_Way==2)  Find_CCD_Zhongzhi();//读取CCD数据提取中线
  if(Flag_Way==3)  ELE_Sensor=(int)(Get_ELE_Bias());//读取电磁巡线传感器偏离中线数据
  Get_RC();
  Kinematic_Analysis(Velocity,Angle);    //小车运动学分析
  Voltage=(int)(ADC_Ave(ADC1,ADC1_SE8,ADC_12bit,10)*33*11*10/4096);
  if(Turn_Off(Voltage)==0)                              		//===如果不存在异常
  {
    Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===速度闭环控制计算电机A最终PWM
    Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===速度闭环控制计算电机B最终PWM
    Xianfu_Pwm();                                                      //===PWM限幅
    Set_Pwm(Motor_A,Motor_B,Servo);                                 	 //===赋值给PWM寄存器
  }
  else
    Set_Pwm(0,0,SERVO_INIT);                                 		//===赋值给PWM寄存器
  Key();    //===扫描按键状态 单击双击可以改变小车运行状态

}

/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=(int)(velocity*(1+T*tan(angle)/2/L));
		Target_B=(int)(velocity*(1-T*tan(angle)/2/L));      //后轮差速
		Servo=(int)(SERVO_INIT+angle*K);                    //舵机转向
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
  if(Flag_Way>=2)//巡线模式下，只允许电机正转
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
函数功能：限制PWM赋值
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(Motor_A<-Amplitude) Motor_A=-Amplitude;
		if(Motor_A>Amplitude)  Motor_A=Amplitude;
	  if(Motor_B<-Amplitude) Motor_B=-Amplitude;
		if(Motor_B>Amplitude)  Motor_B=Amplitude;
		if(Servo<(SERVO_INIT-300))     Servo=SERVO_INIT-300;	  //舵机限幅
		if(Servo>(SERVO_INIT+300))     Servo=SERVO_INIT+300;		  //舵机限幅
}
/**************************************************************************
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=KEY_Scan(100);
	if(tmp==1)Flag_Stop=!Flag_Stop;//单击控制小车的启停
	//if(tmp==2)Flag_Show=!Flag_Show;//双击控制小车的显示状态
	tmp2=Long_Press();
  if(tmp2==1)Flag_Show=!Flag_Show;//控制小车的显示状态
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
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

  if(Flag_Stop==1) temp=1;   //电池电压低于7.4V关闭电机
  else             temp=0;
  return temp;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：遥控
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	int Yuzhi=2;
	static float Bias,Last_Bias;
  float LY,RX;
	if(Flag_Way==0)//蓝牙控制
	{
		if(Flag_Direction==0) Velocity=0,Angle=0;   //停止
		else if(Flag_Direction==1) Velocity=Bluetooth_Velocity,Angle=0;  //前进
		else if(Flag_Direction==2) Velocity=Bluetooth_Velocity,Angle=PI/5;  //右前
		else if(Flag_Direction==3) Velocity=0,Angle=0;   //舵机向右
		else if(Flag_Direction==4) Velocity=-Bluetooth_Velocity,Angle=PI/5;  // 右后
		else if(Flag_Direction==5) Velocity=-Bluetooth_Velocity,Angle=0;    //后退
		else if(Flag_Direction==6) Velocity=-Bluetooth_Velocity,Angle=-PI/5;  //左后
		else if(Flag_Direction==7) Velocity=0,Angle=0;                       //舵机向左
		else if(Flag_Direction==8) Velocity=Bluetooth_Velocity,Angle=-PI/5;  //左前
	}
	else	if(Flag_Way==1)//PS2控制
	{
  	LY=PS2_LY-128;     //计算偏差
		RX=PS2_RX-128;
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0;   //小角度设为死区 防止抖动出现异常
	  if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
		 Velocity=-LY/15;	  //速度和摇杆的力度相关。
		 Angle=RX/200;
	}
		else	if(Flag_Way==2)//CCD巡线
	{
		 Velocity=7;	   //CCD巡线模式的速度
		 Bias=CCD_Zhongzhi-64;   //提取偏差
		 Angle=Bias*0.010+(Bias-Last_Bias)*0.085; //PD控制
		 Last_Bias=Bias;   //保存上一次的偏差
	}
		else	if(Flag_Way==3)//电磁巡线
	{
		 Velocity=7;	  //电磁巡线模式下的速度
		 Bias=100-ELE_Sensor;  //提取偏差
		 Angle=Bias*0.004+(Bias-Last_Bias)*0.02; //非常规PID公式abs((int)(Bias))*Bias*0.0002+
		 Last_Bias=Bias;   //上一次的偏差
	}
}






