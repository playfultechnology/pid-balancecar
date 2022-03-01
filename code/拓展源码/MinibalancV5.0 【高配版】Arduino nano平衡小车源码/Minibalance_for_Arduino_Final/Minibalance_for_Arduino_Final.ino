/****************************************************************************
   作者：平衡小车之家
   产品名称：Minibalance For Arduino
****************************************************************************/
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <KalmanFilter.h>    //卡尔曼滤波
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"  
#include <EEPROM.h>         
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
#define KEY 3     //按键引脚
#define IN1 12   //TB6612FNG驱动模块控制信号 共6个
#define IN2 11
#define IN3 5
#define IN4 8
#define PWMA 9
#define PWMB 10
#define ENCODER_L 2  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 7
#define ENCODER_R 4
#define DIRECTION_R 6
#define ZHONGZHI 0//小车的机械中值  DIFFERENCE
#define DIFFERENCE 2
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
int Motor1, Motor2;      //电机叠加之后的PWM
float Battery_Voltage;   //电池电压 单位是V
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度
int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量
int Angle, Show_Data,PID_Send;  //用于显示的角度和临时变量
unsigned char Flag_Stop = 0,Send_Count,Flash_Send,Receive_Data;  //停止标志位和上位机相关变量
float Balance_Kp=15,Balance_Kd=0.4,Velocity_Kp=2,Velocity_Ki=0.01;
//***************下面是卡尔曼滤波相关变量***************//
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
int addr = 0;
/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：Z轴加速度 平衡倾角 左轮编码器 右轮编码器
返回  值：0：无事件 1：小车被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right){
  static unsigned int flag, count0, count1, count2;
  if (flag == 0) //第一步
  {
    if (abs(encoder_left) + abs(encoder_right) < 15)         count0++;  //条件1，小车接近静止
    else       count0 = 0;
    if (count0 > 10)      flag = 1, count0 = 0;
  }
  if (flag == 1) //进入第二步
  {
    if (++count1 > 400)       count1 = 0, flag = 0;                         //超时不再等待2000ms
    if (Acceleration > 27000 && (Angle > (-14 + ZHONGZHI)) && (Angle < (14 + ZHONGZHI)))  flag = 2; //条件2，小车是在0度附近被拿起
  }
  if (flag == 2)  //第三步
  {
    if (++count2 > 200)       count2 = 0, flag = 0;       //超时不再等待1000ms
    if (abs(encoder_left + encoder_right) > 300)           //条件3，小车的轮胎因为正反馈达到最大的转速      
     {
        flag = 0;  return 1;
      }                                           
  }
  return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下 作者：平衡小车之家
入口参数： 平衡倾角 左轮编码器 右轮编码器
返回  值：0：无事件 1：小车放置并启动
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right){
  static u16 flag, count;
  if (Flag_Stop == 0)         return 0;                   //防止误检
  if (flag == 0)
  {
    if (Angle > (-10 + ZHONGZHI) && Angle < (10 + ZHONGZHI) && encoder_left == 0 && encoder_right == 0)      flag = 1; //条件1，小车是在0度附近的
  }
  if (flag == 1)
  {
    if (++count > 100)       count = 0, flag = 0;  //超时不再等待 500ms
    if (encoder_left > 12 && encoder_right > 12 && encoder_left < 80 && encoder_right < 80) //条件2，小车的轮胎在未上电的时候被人为转动
    {
      flag = 0;
      flag = 0;
      return 1;    //检测到小车被放下
    }
  }
  return 0;
}
/**************************************************************************
函数功能：异常关闭电机 作者：平衡小车之家
入口参数：倾角和电池电压
返回  值：1：异常  0：正常
**************************************************************************/
unsigned char Turn_Off(float angle, float voltage)
{
  unsigned char temp;
  if (angle < -40 || angle > 40 || 1 == Flag_Stop || voltage < 11.1) //电池电压低于11.1V关闭电机 //===倾角大于40度关闭电机//===Flag_Stop置1关闭电机
  {                                                                         
    temp = 1;                                          
    analogWrite(PWMA, 0);  //PWM输出为0
    analogWrite(PWMB, 0); //PWM输出为0
  }
  else    temp = 0;   //不存在异常，返回0
  return temp;
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(Angle, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(Velocity_Left, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(Velocity_Right, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(Battery_Voltage, 4);//显示第四个数据，电池电压，单位V
  Send_Count = data.DataScope_Data_Generate(4); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}

/**************************************************************************
函数功能：直立PD控制  作者：平衡小车之家
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
  float Bias;
  int balance;
  Bias = Angle - 0;   //===求出平衡的角度中值 和机械相关
  balance = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance;
}
/**************************************************************************
函数功能：速度PI控制 作者：平衡小车之家
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
  float kp = 2, ki = kp / 200;    //PI参数
  if       ( Flag_Qian == 1)Movement = 600;
  else   if ( Flag_Hou == 1)Movement = -600;
  else    //这里是停止的时候反转，让小车尽快停下来
  {
    Movement = 0;
    if (Encoder_Integral > 300)   Encoder_Integral -= 200;
    if (Encoder_Integral < -300)  Encoder_Integral += 200;
  }
  //=============速度PI控制器=======================//
  Encoder_Least = (encoder_left + encoder_right) - 0;               //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.7;                                                   //===一阶低通滤波器
  Encoder += Encoder_Least * 0.3;                                   //===一阶低通滤波器
  Encoder_Integral += Encoder;                                      //===积分出位移 积分时间：40ms
  Encoder_Integral = Encoder_Integral - Movement;                   //===接收遥控器数据，控制前进后退
  if (Encoder_Integral > 21000)    Encoder_Integral = 21000;        //===积分限幅
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;         //===积分限幅
  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;                  //===速度控制
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//小车停止的时候积分清零
  return Velocity;
}
/**************************************************************************
函数功能：转向控制 作者：平衡小车之家
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(float gyro)//转向控制
{
  static float Turn_Target, Turn, Turn_Convert = 3;
  float Turn_Amplitude = 80, Kp = 2, Kd = 0.001;  //PD参数
  if (1 == Flag_Left)             Turn_Target += Turn_Convert;  //根据遥控指令改变转向偏差
  else if (1 == Flag_Right)       Turn_Target -= Turn_Convert;//根据遥控指令改变转向偏差
  else Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude; //===转向速度限幅
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
  Turn = -Turn_Target * Kp + gyro * Kd;         //===结合Z轴陀螺仪进行PD控制
  return Turn;
}
/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
  if (moto1 < 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  //TB6612的电平控制
  else             digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); //TB6612的电平控制
  analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器
  if (moto2 > 0) digitalWrite(IN3, HIGH),     digitalWrite(IN4, LOW); //TB6612的电平控制
  else        digitalWrite(IN3, LOW),      digitalWrite(IN4, HIGH); //TB6612的电平控制
  analogWrite(PWMB, abs(moto2));//赋值给PWM寄存器
}
/**************************************************************************
函数功能：限制PWM赋值  作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if(Flag_Qian==1)  Motor2-=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
  if(Flag_Hou==1)   Motor2-=DIFFERENCE-2;
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude)  Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude)  Motor2 = Amplitude;
}
/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{
  static int Velocity_Count, Turn_Count, Encoder_Count;
  static float Voltage_All,Voltage_Count;
  int Temp;
  sei();//全局中断开启
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);          //通过卡尔曼滤波获取角度
  Angle = KalFilter.angle;//Angle是一个用于显示的整形变量
  Balance_Pwm = balance(KalFilter.angle, KalFilter.Gyro_x);//直立PD控制 控制周期5ms
  if (++Velocity_Count >= 8) //速度控制，控制周期40ms
  {
    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);//速度PI控制，控制周期40ms
    Velocity_Count = 0;
  }
  if (++Turn_Count >= 4)//转向控制，控制周期20ms
  {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;  //直立速度转向环的叠加
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm; //直立速度转向环的叠加
  Xianfu_Pwm();//限幅
  if (Pick_Up(az, KalFilter.angle, Velocity_Left, Velocity_Right))   Flag_Stop = 1;  //===如果被拿起就关闭电机//===检查是否小车被那起
  if (Put_Down(KalFilter.angle, Velocity_Left, Velocity_Right))      Flag_Stop = 0;              //===检查是否小车被放下
 if (Turn_Off(KalFilter.angle, Battery_Voltage) == 0)  
  Set_Pwm(Motor1, Motor2);//如果不存在异常，赋值给PWM寄存器控制电机
  Temp = analogRead(7);  //采集一下电池电压
  Voltage_Count++;       //平均值计数器
  Voltage_All+=Temp;     //多次采样累积
  if(Voltage_Count==200) Battery_Voltage=Voltage_All*0.05371/200,Voltage_All=0,Voltage_Count=0;//求平均值
}
/**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup() {
  pinMode(IN1, OUTPUT);        //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
  pinMode(IN2, OUTPUT);          //TB6612控制引脚，
  pinMode(IN3, OUTPUT);          //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
  pinMode(IN4, OUTPUT);          //TB6612控制引脚，
  pinMode(PWMA, OUTPUT);         //TB6612控制引脚，电机PWM
  pinMode(PWMB, OUTPUT);         //TB6612控制引脚，电机PWM
  digitalWrite(IN1, 0);          //TB6612控制引脚拉低
  digitalWrite(IN2, 0);          //TB6612控制引脚拉低
  digitalWrite(IN3, 0);          //TB6612控制引脚拉低
  digitalWrite(IN4, 0);          //TB6612控制引脚拉低
  analogWrite(PWMA, 0);          //TB6612控制引脚拉低
  analogWrite(PWMB, 0);          //TB6612控制引脚拉低
  pinMode(2, INPUT);       //编码器引脚
  pinMode(4, INPUT);       //编码器引脚
  pinMode(6, INPUT);       //编码器引脚
  pinMode(7, INPUT);       //编码器引脚
  pinMode(3, INPUT);       //按键引脚
  Wire.begin();             //加入 IIC 总线
  Serial.begin(9600);       //开启串口，设置波特率为 9600
  delay(1500);              //延时等待初始化完成
  Mpu6050.initialize();     //初始化MPU6050
  delay(20); 
MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
 MsTimer2::start();          //使用中断使能
  attachInterrupt(0, READ_ENCODER_L, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, READ_ENCODER_R, CHANGE);  //开启外部中断 编码器接口2
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() { 
  int Voltage_Temp;
  static unsigned char flag;
  unsigned char Balance_Kp_Temp=0,Balance_Kd_Temp=0,Velocity_Kp_Temp=0,Velocity_Ki_Temp=0;
  Voltage_Temp = (Battery_Voltage - 11.1) * 60;  //根据APP的协议对电池电压变量进行处理
  if (Voltage_Temp > 100)Voltage_Temp = 100;
  if (Voltage_Temp < 0)Voltage_Temp = 0;
  if (Flag_Stop == 0)
  {
    Serial.begin(9600);       //开启串口，设置波特率为 9600
    flag=!flag;
      if(PID_Send==1)//发送PID参数
  {
    Serial.print("{C");
    Serial.print((int)(Balance_Kp*100));   //左轮编码器
    Serial.print(":");
    Serial.print((int)(Balance_Kd*100));  //右轮编码器
    Serial.print(":");
    Serial.print((int)(Velocity_Kp*100));  //电池电压
    Serial.print(":");
    Serial.print((int)(Velocity_Ki*100));  //平衡倾角
    Serial.print("}$");
    PID_Send=0; 
  } 
    else if(flag==0)
    {
    Serial.print("{A");
    Serial.print(abs(Velocity_Left / 2));   //左轮编码器
    Serial.print(":");
    Serial.print(abs(Velocity_Right / 2));  //右轮编码器
    Serial.print(":");
    Serial.print(Voltage_Temp);  //电池电压
    Serial.print(":");
    Serial.print(Angle);  //平衡倾角
    Serial.print("}$");
    }
      else
    {
    Serial.print("{B");
    Serial.print(Angle);   
    Serial.print(":");
    Serial.print(Voltage_Temp);  
    Serial.print(":");
    Serial.print(Velocity_Left/2); 
    Serial.print(":");
    Serial.print(Velocity_Right/2); 
    Serial.print("}$");
    }
    delay(50);
  }
  else    Serial.begin(128000), DataScope(); //使用上位机时，波特率是128000
    if(Flash_Send==1)        //写入PID参数到EEPROM,由app控制该指令
    {
     EEPROM.write(addr,     ((unsigned int)(Balance_Kp*100)&0xff00)>>8);
     EEPROM.write(addr+1,   (unsigned int)(Balance_Kp*100)&0xff);
     EEPROM.write(addr+2,   ((unsigned int)(Balance_Kd*100)&0xff00)>>8);
     EEPROM.write(addr+3,   (unsigned int)(Balance_Kd*100)&0xff);
     EEPROM.write(addr+4,   ((unsigned int)(Velocity_Kp*100)&0xff00)>>8);
     EEPROM.write(addr+5,   (unsigned int)(Velocity_Kp*100)&0xff);
     EEPROM.write(addr+6,   ((unsigned int)(Velocity_Ki*100)&0xff00)>>8);
     EEPROM.write(addr+7,   (unsigned int)(Velocity_Ki*100)&0xff);
     Flash_Send=0; 
    } 
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;//根据另外一相电平判定方向
    else      Velocity_R--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--; //根据另外一相电平判定方向
    else     Velocity_R++;
  }
}
/**************************************************************************
函数功能：串口接收中断
入口参数：无
返回  值：无
**************************************************************************/
void serialEvent() 
{    
    static unsigned char Flag_PID,Receive[10],i,j;
   static float Data;
    while (Serial.available()) {
    Receive_Data=Serial.read();  
    if(Receive_Data==0x7B) Flag_PID=1;  //参数指令起始位
    if(Receive_Data==0x7D) Flag_PID=2;  //参数指令停止位
    if(Flag_PID==1)
     {
      Receive[i]=Receive_Data;     //记录数据
      i++;
     }
    else  if(Flag_PID==2)  //执行指令
     {
       if(Receive[3]==0x50)          PID_Send=1;   //获取PID参数
       else  if(Receive[3]==0x57)    Flash_Send=1; //掉电保存参数
       else  if(Receive[1]!=0x23)    //更新PID参数
       {                
        for(j=i;j>=4;j--)
        {
          Data+=(Receive[j-1]-48)*pow(10,i-j);   //通讯协议
        }
        switch(Receive[1])
         {
           case 0x30:  Balance_Kp=Data/100;break;
           case 0x31:  Balance_Kd=Data/100;break;
           case 0x32:  Velocity_Kp=Data/100;break;
           case 0x33:  Velocity_Ki=Data/100;break;
           case 0x34:  break; //9个通道，预留5个
           case 0x35:  break;
           case 0x36:  break;
           case 0x37:  break;
           case 0x38:  break;
         }
       }         
           Flag_PID=0; //相关标志位清零
           i=0;
           j=0;
           Data=0;
     }
       else  //蓝牙遥控指令
       {
              switch (Receive_Data)   {
                 //这是MinibalanceV1.0的APP发送指令
                  case 0x01: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
                  case 0x02: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
                  case 0x03: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
                  case 0x04: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
                  case 0x05: Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;              //后退
                  case 0x06: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
                  case 0x07: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
                  case 0x08: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
                  //这是MinibalanceV3.5的APP发送指令
                  case 0x41: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
                  case 0x42: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
                  case 0x43: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
                  case 0x44: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
                  case 0x45:  Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;             //后退
                  case 0x46: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;    break;               //左转
                  case 0x47: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
                  case 0x48: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;             //左转
                  default: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;    break;                //停止
                }
        }
   }
}
