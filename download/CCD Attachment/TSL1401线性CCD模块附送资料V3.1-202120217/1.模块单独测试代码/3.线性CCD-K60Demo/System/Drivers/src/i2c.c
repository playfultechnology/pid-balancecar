/**
  ******************************************************************************
  * @file    i2c.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� IIC �����ļ�
  ******************************************************************************
  */
#include "i2c.h"

/***********************************************************************************************
 ���ܣ�I2C ��ʼ��
 �βΣ�I2C_InitStruct: I2C��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void I2C_Init(I2C_InitTypeDef* I2C_InitStruct)
{
	I2C_Type *I2Cx = NULL;
	PORT_Type *I2C_PORT = NULL;
	uint32_t prescaler = 0;
	I2C_MapTypeDef *pI2C_Map = (I2C_MapTypeDef*)&(I2C_InitStruct->I2CxMAP);
	//�������
	assert_param(IS_I2C_DATA_CHL(I2C_InitStruct->I2CxMAP));
	assert_param(IS_I2C_CLOCK_SPEED(I2C_InitStruct->I2C_ClockSpeed));
	//ʹ��I2Cʱ��
	switch(pI2C_Map->I2C_Index)
	{
		case 0:
			SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK; 
			I2Cx = I2C0;
			break;
		case 1:
			SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
			I2Cx = I2C1;		
			break;
		default:break;
	}
	//ʹ�ܶ�Ӧ��PORT
	switch(pI2C_Map->I2C_GPIO_Index)
	{
		case 0:
			SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
			I2C_PORT = PORTA;
			break;
		case 1:
			SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			I2C_PORT = PORTB;
			break;
		case 2:
			SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
			I2C_PORT = PORTC;
			break;
		case 3:
			SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
			I2C_PORT = PORTD;
			break;
		case 4:
			SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			I2C_PORT = PORTE;
			break;
		default:break;
	}
	//������Ӧ����ΪI2C����
	I2C_PORT->PCR[pI2C_Map->I2C_SCL_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	I2C_PORT->PCR[pI2C_Map->I2C_SDA_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	I2C_PORT->PCR[pI2C_Map->I2C_SCL_Pin_Index] |= PORT_PCR_MUX(pI2C_Map->I2C_Alt_Index)|PORT_PCR_ODE_MASK;
	I2C_PORT->PCR[pI2C_Map->I2C_SDA_Pin_Index] |= PORT_PCR_MUX(pI2C_Map->I2C_Alt_Index)|PORT_PCR_ODE_MASK;
	//����������Ϊ©�����
	//����I2C��Ƶ��
	prescaler = (((CPUInfo.BusClock /(I2C_InitStruct->I2C_ClockSpeed))-160))/32 +  0x20;
	I2Cx->F	= prescaler;
	//ʹ��I2Cģ��
	I2Cx->C1 = I2C_C1_IICEN_MASK ;
}

/***********************************************************************************************
 ���ܣ�I2C ���Ϳ�ʼ�ź�
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ�0
 ��⣺ I2C ���ߵ�SCLΪ�ߵ�ƽ ʱ SDA�½������� ʶ��Ϊ��ʼ�ź�
************************************************************************************************/
void I2C_GenerateSTART(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->C1 |= I2C_C1_TX_MASK;
	I2Cx->C1 |= I2C_C1_MST_MASK;
}
/***********************************************************************************************
 ���ܣ�I2C �������¿�ʼ�ź�
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ�0
 ��⣺ I2C ���ߵ�SCLΪ�ߵ�ƽ ʱ SDA�½������� ʶ��Ϊ��ʼ�ź�
************************************************************************************************/
void I2C_GenerateRESTART(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->C1 |= I2C_C1_RSTA_MASK;
}
/***********************************************************************************************
 ���ܣ�I2C ����ֹͣ�ź�
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ�0
 ��⣺ I2C ���ߵ�SCLΪ�ߵ�ƽ ʱ SDA���������� ʶ��Ϊ�����ź�
************************************************************************************************/
void I2C_GenerateSTOP(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->C1 &= ~I2C_C1_MST_MASK;
	I2Cx->C1 &= ~I2C_C1_TX_MASK;
}
/***********************************************************************************************
 ���ܣ�I2C ����8bit����
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
			 data8: 8bit����
 ���أ�0
 ��⣺0
************************************************************************************************/
void I2C_SendData(I2C_Type *I2Cx,uint8_t data8)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->D = data8;
}
/***********************************************************************************************
 ���ܣ�I2C ��ȡ8bit����
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ���ȡ����8bit ����
 ��⣺0
************************************************************************************************/
uint8_t I2C_ReadData(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	return (I2Cx->D);
}
/***********************************************************************************************
 ���ܣ�I2C ����7λ��ַ��
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
			 Address: 7bit��ַ��
       I2C_Direction��
       @arg I2C_MASTER_WRITE: ����д
       @arg I2C_MASTER_READ:  ������
 ���أ�0
 ��⣺��װ��SendData
************************************************************************************************/
void I2C_Send7bitAddress(I2C_Type* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_MASTER_DIRECTION(I2C_Direction));
	
	(I2C_Direction == I2C_MASTER_WRITE)?(Address &= 0xFE):(Address |= 0x01);
	I2Cx->D = Address;
}
/***********************************************************************************************
 ���ܣ�I2C �ȴ�Ӧ���ź����
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��

 ���أ�0   :�ɹ��յ�Ӧ�� 
       1   :δ�յ�Ӧ��
 ��⣺����ÿ����һ���ֽڶ���Ҫ����I2C_WaitAckһ�� WaitAck ��ȴ���һ���ֽڴ������ �������ACKλ�Ľ����󷵻�
************************************************************************************************/
uint8_t I2C_WaitAck(I2C_Type *I2Cx)
{

	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
    //wait for transfer complete
    while ((I2Cx->S & I2C_S_TCF_MASK) == 0);
    //both TCF and IICIF indicate one byte trasnfer complete
    while ((I2Cx->S & I2C_S_IICIF_MASK) == 0);
	  //IICIF is a W1C Reg, so clear it!
	  I2Cx->S |= I2C_S_IICIF_MASK;
    //see if we receive the ACK signal
    if (I2Cx->S & I2C_S_RXAK_MASK)
		{
        return 1;
		}
		else
		{
        return 0;
		}
}

/***********************************************************************************************
 ���ܣ�I2C ����������дģʽ
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
			 I2C_Direction : ������д����
			 @arg  I2C_MASTER_WRITE : ����д
       @arg  I2C_MASTER_READ  : ������
 ���أ�0
 ��⣺0 
************************************************************************************************/
void I2C_SetMasterMode(I2C_Type* I2Cx,uint8_t I2C_Direction)
{
	//�������
	assert_param(IS_I2C_MASTER_DIRECTION(I2C_Direction));
	
	(I2C_Direction == I2C_MASTER_WRITE)?(I2Cx->C1 |= I2C_C1_TX_MASK):(I2Cx->C1 &= ~I2C_C1_TX_MASK);
}
/***********************************************************************************************
 ���ܣ�I2C ����Ϊ��ȡһ���ֽں󷵻�NACK
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ�0
 ��⣺0 
************************************************************************************************/
void I2C_GenerateNAck(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->C1 |= I2C_C1_TXAK_MASK;
}
/***********************************************************************************************
 ���ܣ�I2C ����Ϊ��ȡһ���ֽں󷵻�ACK
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0: I2C0 ģ��
       @arg I2C1: I2C1 ģ��
 ���أ�0
 ��⣺0 
************************************************************************************************/
void I2C_GenerateAck(I2C_Type *I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	I2Cx->C1 &= ~I2C_C1_TXAK_MASK;
}

/***********************************************************************************************
 ���ܣ�I2C �ж�����
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
			 I2C_IT : �ж�Դ
       @arg I2C_IT_TCF : I2C�жϴ������
       NewState : ʹ�ܻ��߽�ֹ
       @arg ENABLE : ʹ��
       @arg DISABLE: ��ֹ 
 ���أ�0
 ��⣺0
************************************************************************************************/
void I2C_ITConfig(I2C_Type* I2Cx, uint16_t I2C_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_IT(I2C_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(I2C_IT)
	{
		case I2C_IT_TCF:
			(ENABLE == NewState)?(I2Cx->C1 |= I2C_C1_IICIE_MASK):(I2Cx->C1 &= ~I2C_C1_IICIE_MASK);
			break;
		case I2C_IT_IAAS:
			(ENABLE == NewState)?(I2Cx->C1 |= I2C_C1_IICIE_MASK):(I2Cx->C1 &= ~I2C_C1_IICIE_MASK);
		  (ENABLE == NewState)?(I2Cx->C1 |= I2C_C1_WUEN_MASK):(I2Cx->C1 &= ~I2C_C1_WUEN_MASK);
			break;
		case I2C_IT_SLTF:
			(ENABLE == NewState)?(I2Cx->C1 |= I2C_C1_IICIE_MASK):(I2Cx->C1 &= ~I2C_C1_IICIE_MASK);
			break;
		case I2C_IT_SHTF2:
			(ENABLE == NewState)?(I2Cx->C1 |= I2C_C1_IICIE_MASK):(I2Cx->C1 &= ~I2C_C1_IICIE_MASK);
		  (ENABLE == NewState)?(I2Cx->SMB |= I2C_SMB_SHTF2IE_MASK):(I2Cx->SMB &= ~I2C_SMB_SHTF2IE_MASK);
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�I2C ��ȡ�ж�״̬
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
			 I2C_IT : �ж�Դ
       @arg I2C_IT_TCF : I2C�жϴ������
 ���أ�0
 ��⣺0
************************************************************************************************/
ITStatus I2C_GetITStatus(I2C_Type* I2Cx, uint16_t I2C_IT)
{
	ITStatus retval = RESET;
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_IT(I2C_IT));
	switch(I2C_IT)
	{
		case I2C_IT_TCF:
			(I2Cx->S & I2C_S_TCF_MASK)?(retval = SET):(retval = RESET);
			break;
		case I2C_IT_IAAS:
			(I2Cx->S & I2C_S_IAAS_MASK)?(retval = SET):(retval = RESET);
			break;
		case I2C_IT_SLTF:
			(I2Cx->SMB & I2C_SMB_SLTF_MASK)?(retval = SET):(retval = RESET);
			break;
		case I2C_IT_ARBL:
			(I2Cx->S & I2C_S_ARBL_MASK)?(retval = SET):(retval = RESET);
			break;  
		case I2C_IT_SHTF2:
			(I2Cx->SMB & I2C_SMB_SHTF2_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}

/***********************************************************************************************
 ���ܣ�I2C DMAʹ��
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
			 I2C_DMAReq : �ж�Դ
       @arg I2C_DMAReq_TCF : I2C�������
       NewState : ʹ�ܻ��߽�ֹ
       @arg ENABLE : ʹ��
       @arg DISABLE: ��ֹ 
 ���أ�0
 ��⣺0
************************************************************************************************/
void I2C_DMACmd(I2C_Type* I2Cx, uint16_t I2C_DMAReq, FunctionalState NewState)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_DMAREQ(I2C_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(I2C_DMAReq)
	{
		case I2C_DMAReq_TCF:
			(NewState == ENABLE)?(I2Cx->C1 |= I2C_C1_DMAEN_MASK):(I2Cx->C1 &= ~I2C_C1_DMAEN_MASK);
			break;
		default:break;
	}
}

/***********************************************************************************************
 ���ܣ�I2C ����жϱ�־
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
			 I2C_IT : �ж�Դ
       @arg I2C_IT_TCF : I2C�жϴ������
 ���أ�0
 ��⣺0
************************************************************************************************/
void I2C_ClearITPendingBit(I2C_Type* I2Cx, uint16_t I2C_IT)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_IT(I2C_IT));
	
	//���жϱ�־λ
	I2Cx->C1 |= I2C_S_IICIF_MASK;
	switch(I2C_IT)
	{
		case I2C_IT_TCF:
			break;
		case I2C_IT_IAAS:
			break;
		case I2C_IT_SLTF:
			I2Cx->SMB |= I2C_SMB_SLTF_MASK;
			break; 
		case I2C_IT_ARBL:
	  	I2Cx->S |= I2C_S_ARBL_MASK;
			break;
		case I2C_IT_SHTF2:
			I2Cx->SMB |= I2C_SMB_SHTF2_MASK;
			break;
	}
}
/***********************************************************************************************
 ���ܣ��ж�I2C �����Ƿ���� �ߵ�ƽ
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
 ���أ�TRUE: ���� FALSE :æ
 ��⣺0
************************************************************************************************/
uint8_t I2C_IsLineBusy(I2C_Type* I2Cx)
{
	//�������
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	if(I2Cx->S & I2C_S_BUSY_MASK)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

uint8_t I2C_Write(I2C_Type *I2Cx ,uint8_t DeviceAddress, uint8_t *pBuffer, uint32_t len)
{
    //Generate START signal
    I2C_GenerateSTART(I2Cx);
    //Send 7bit Data with WRITE operation
    I2C_Send7bitAddress(I2Cx, DeviceAddress, I2C_MASTER_WRITE);
    if(I2C_WaitAck(I2Cx))
    {
			  I2C_GenerateSTOP(I2Cx);
			  while((I2Cx->S & I2C_S_BUSY_MASK) == 1) {};
        return 1;
    }
		//Send All Data
		while(len--)
		{
        I2C_SendData(I2Cx, *(pBuffer++));
        if(I2C_WaitAck(I2Cx))
        {
            I2C_GenerateSTOP(I2Cx);
            while((I2Cx->S & I2C_S_BUSY_MASK) == 1) {};
            return 2;
        }
		}
		//Generate stop and wait for line idle
		I2C_GenerateSTOP(I2Cx);
    while((I2Cx->S & I2C_S_BUSY_MASK) == 1) {};
		return 0;
}


/***********************************************************************************************
 ���ܣ�ͨ�� I2C ��дһ���ӻ���һ���Ĵ��� �����ڴ�����ڼ� MMA84ϵ�д������ȵ�
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
       DeviceAddress: �豸��ַ
       RegisterAddress:�Ĵ������豸�еĵ�ַ
       Data: ��Ҫд�������
 ���أ�0:�ɹ� else: �������
 ��⣺ʵ��˳��Ϊ START->ADDRESS->RegADR->Data->STOP->Wait until all stop
************************************************************************************************/
uint8_t I2C_WriteSingleRegister(I2C_Type* I2Cx, uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t Data)
{
    uint8_t ret;
    uint8_t buf[2];
    buf[0] = RegisterAddress;
    buf[1] = Data;
    ret = I2C_Write(I2Cx, DeviceAddress, buf, sizeof(buf));
    return ret;
}



/***********************************************************************************************
 ���ܣ�ͨ�� I2C ��дһ���ӻ���һ���Ĵ��� �����ڴ�����ڼ� MMA84ϵ�д������ȵ�
 �βΣ�I2Cx: I2Cģ���
       @arg I2C0 : I2C0ģ��
       @arg I2C1 : I2C1ģ��
       DeviceAddress: �豸��ַ
       RegisterAddress:�Ĵ������豸�еĵ�ַ
       Data: ��Ҫд�������
 ���أ�0:�ɹ� else: �������
 ��⣺ʵ��˳��Ϊ START->ADDRESS->RESTART->ADDRESS_With_READ->ReadData->SEND_NACK->STOP->Wait until all stop
************************************************************************************************/
uint8_t I2C_ReadSingleRegister(I2C_Type* I2Cx, uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t* pData)
{
    uint8_t data;
    //Generate START signal
    I2C_GenerateSTART(I2Cx);
    //Send 7bit Data with WRITE operation
    I2C_Send7bitAddress(I2Cx, DeviceAddress, I2C_MASTER_WRITE);
    if(I2C_WaitAck(I2Cx))
    {
			  I2C_GenerateSTOP(I2Cx);
			  while((I2Cx->S & I2C_S_BUSY_MASK) == 1);
        return 1;
    }
    //Send Reg Address
		I2C_SendData(I2Cx, RegisterAddress);
    if(I2C_WaitAck(I2Cx))
    {
			  I2C_GenerateSTOP(I2Cx);
			  while((I2Cx->S & I2C_S_BUSY_MASK) == 1);
        return 2;
    }
		//Generate RESTART Signal
    I2C_GenerateRESTART(I2Cx);
		//Resend 7bit Address, This time we use READ Command
    I2C_Send7bitAddress(I2Cx, DeviceAddress, I2C_MASTER_READ);
    if(I2C_WaitAck(I2Cx))
    {
			  I2C_GenerateSTOP(I2Cx);
        while((I2Cx->S & I2C_S_BUSY_MASK) == 1);
        return 3;
    }
    //Set Master in slave mode
    I2C_SetMasterMode(I2Cx,I2C_MASTER_READ);
		//Dummy Read in order to generate SCL clock
    data = I2Cx->D;
		I2C_GenerateNAck(I2Cx);
		//This time, We just wait for masters receive byte transfer complete
    I2C_WaitAck(I2Cx);
		//Generate stop and wait for line idle
		I2C_GenerateSTOP(I2Cx);
    while((I2Cx->S & I2C_S_BUSY_MASK) == 1);
		//actual read
		data = I2Cx->D;
		*pData = data;
		return 0;
}

/*
static const I2C_MapTypeDef I2C_Check_Maps[] = 
{ 
    {1, 4, 6, 1, 0,0},  //I2C1_SCL_PE1_SDA_PE0
    {0, 1, 2, 0, 1,0},  //I2C0_SCL_PB0_SDA_PB1
    {0, 1, 2, 2, 3,0},  //I2C0_SCL_PB2_SDA_PB3
    {1, 2, 2,10,11,0},  //I2C1_SCL_PC10_SDA_PC11
};
void I2C_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(I2C_Check_Maps)/sizeof(I2C_MapTypeDef);i++)
	{
		value = I2C_Check_Maps[i].I2C_Index<<0;
		value|= I2C_Check_Maps[i].I2C_GPIO_Index<<4;
		value|= I2C_Check_Maps[i].I2C_Alt_Index<<8;
		value|= I2C_Check_Maps[i].I2C_SCL_Pin_Index<<12;
		value|= I2C_Check_Maps[i].I2C_SDA_Pin_Index<<18;
		printf("(0x%08xU)\r\n",value);
	}
}
*/


