#ifndef __IIC_H
#define __IIC_H
#include "common.h"
#include "MK60_GPIO.h"

#define IIC_SCL_PIN  PTC10 //ģ��IIC��SCL�ź�
#define IIC_SDA_PIN  PTC11 //ģ��IIC��SDA�ź�

#define SDA_IN()  GPIO_PinSetDir(IIC_SDA_PIN, 0);	//����
#define SDA_OUT() GPIO_PinSetDir(IIC_SDA_PIN, 1);	//���


#define IIC_SCL    PTC10_OUT  //SCL
#define IIC_SDA    PTC11_OUT  //SDA
#define READ_SDA   PTC11_IN   //����SDA



void IIC_Start(void);			//����IIC��ʼ�ź�
void IIC_Stop(void);	  	        //����IICֹͣ�ź�
void IIC_Ack(void);			//IIC����ACK�ź�
void IIC_NAck(void);			//IIC������ACK�ź�
uint8_t IIC_WaitAck(void); 		//IIC�ȴ�ACK�ź�
void IIC_SendByte(uint8_t data);        //IIC����һ���ֽ�
uint8_t IIC_ReadByte(uint8_t ack);      //IIC��ȡһ���ֽ�
void IIC_Delay(uint8_t us);             //IICģ��IO��ת��ʱ


void IIC_Init(void);                    //��ʼ��IIC��IO��
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif