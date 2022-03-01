#include "control.h"	
#include "filter.h"	
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Velocity=1;
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
u32 Run_Count;
int Voltage_Temp,Voltage_Count,Voltage_All,Key_Count;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{   
		  EXTI->PR=1<<15;                                                      //清除中断标志位   
	  	Flag_Target=!Flag_Target;
		   if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 }
			 if(Run_Count++>300)     //3s之后才执行控制代码
			 {
					 if( Flag_Target==0)  
					 {
							Get_Angle(1);  //===更新姿态	
							return 0;	  
					 }
						if(Menu_MODE==1)		//位置控制
					{
						Encoder=Read_Position(2);                      //===更新位置信息
						Moto=Position_PID(Encoder,Target_Position);    //===位置PID控制器
					}
					else if(++Flag_Velocity>2)                       //===速度控制20ms一次，也可10ms
					{
						Flag_Velocity=1;
						Encoder=Read_Velocity(2);                      //===更新速度信息
						if(Encoder<0) Encoder=0;                       
						Moto=Incremental_PI(Encoder,Target_Velocity);      //===速度PI控制器
					}
						Encoder_Key=-Read_Velocity(4);   //读取右路编码器的数据控制PID参数调节
					  if(++Key_Count>20) //200ms调节一次
						Key_Count=0,
						PID_Adjust(Encoder_Key);//调节PID参数
	  	 }
	    ///下面4行主要是电池电量采集
			Voltage_Temp=Get_battery_volt();		                                //=====读取电池电压		
			Voltage_Count++;                                                    //=====平均值计数器
			Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====求平均值		                                                                 
	  	Get_Angle(1);                                                       //===更新姿态	
  		Led_Flash(100);                                                     //===LED闪烁;常规模式 1s改变一次指示灯的状态	
			Key();                                                              //===扫描按键状态 单击双击可以改变小车运行状态

   		Xianfu_Pwm();                                                       //===PWM限幅
      if(Turn_Off(Angle_Balance,Voltage)==0)                              //===如果不存在异常
 			Set_Pwm(0,Moto);                                                    //===赋值给PWM寄存器  
	}       	
	 return 0;	  
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
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
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	int tmp,tmp2,Position_Amplitude=390,Velocity_Amplitude=10; 
	static u8 mode=0;
	tmp=click_N_Double(100); 
	 if(Menu_MODE==1)  //改变运行位置 
	 {
		 	if(tmp==1)Target_Position+=Position_Amplitude;
			if(tmp==2)Target_Position-=Position_Amplitude;
	 }
	 else             //改变运行速度
	 {
			if(tmp==1)Target_Velocity+=Velocity_Amplitude;
			if(tmp==2)Target_Velocity-=Velocity_Amplitude;
	 }
	 if(Target_Velocity>70)Target_Velocity=70; //速度最大值限幅
	 if(Target_Velocity<0)Target_Velocity=0;   //速度最小值限幅
	 if(Menu_MODE==1)  mode=3;
	 else mode=2;
   tmp2=Long_Press();        //长按用户按键，选择需要修改的参数
   if(tmp2==1) 
	 {
		 Menu_PID++;
	  if(Menu_PID>mode)  Menu_PID=1;
	 }		
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
				if(angle<-40||angle>40||voltage<1110)//电池电压低于11.1V关闭电机
			{	                                                 //===倾角大于40度关闭电机
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
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
			{	
					Read_DMP();                      //===读取加速度、角速度、倾角
					Angle_Balance=-Roll;             //===更新平衡倾角		
			}			
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
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
int Incremental_PI (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                                     //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000) Integral_bias=100000;
	 if(Integral_bias<-100000) Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias/10+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
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
