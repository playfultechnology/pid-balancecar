#ifndef __IIC_H
#define __IIC_H
#include "common.h" 
#include "MK60_GPIO.h"

#define IIC_SCL_PIN  PTE7 //模拟IIC的SCL信号  1.修改引脚即可修改IIC接口
#define IIC_SDA_PIN  PTE8 //模拟IIC的SDA信号

#define SDA_IN()  GPIO_PinSetDir(IIC_SDA_PIN, 0);	//输入
#define SDA_OUT() GPIO_PinSetDir(IIC_SDA_PIN, 1);	//输出

	 
#define IIC_SCL    PTE7_OUT  //SCL            2.修改引脚即可修改IIC接口    
#define IIC_SDA    PTE8_OUT  //SDA	 
#define READ_SDA   PTE8_IN   //输入SDA  


/*---------------------------------------------------------------
            IIC内部函数
----------------------------------------------------------------*/		 
void IIC_Start(void);			        //发送IIC开始信号
void IIC_Stop(void);	  	            //发送IIC停止信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				    //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		        //IIC等待ACK信号
void IIC_SendByte(uint8_t data);        //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);       //IIC读取一个字节


/*---------------------------------------------------------------
            IIC用户函数
----------------------------------------------------------------*/
void IIC_Init(void);                    //初始化IIC的IO口   
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif