

int Sensor_Left, Sensor_Middle, Sensor_Right, Sensor; //电磁巡线相关


/***********函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家************/
void setup()   {
  Serial.begin(9600);            //蓝牙控制    //开启串口
}
/******函数功能：主循环程序体*******/
void loop(){
  int sum;
  
    Sensor_Left =   analogRead(1);    // delay(2);
    Sensor_Middle = analogRead(2);    // delay(1);
    Sensor_Right =  analogRead(3);
    if (Sensor_Left + Sensor_Middle + Sensor_Right > 25) {
    sum = Sensor_Left * 1 + Sensor_Middle * 50 + Sensor_Right * 99; //归一化处理
    Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right); //求偏差
      }
     Serial.print("      Left:");
     Serial.print(Sensor_Left);
     Serial.print("      Middle:");
     Serial.print(Sensor_Middle);
     Serial.print("      Right:");
     Serial.print(Sensor_Right);
     Serial.print("      Sensor:");
     Serial.println(Sensor);
}


