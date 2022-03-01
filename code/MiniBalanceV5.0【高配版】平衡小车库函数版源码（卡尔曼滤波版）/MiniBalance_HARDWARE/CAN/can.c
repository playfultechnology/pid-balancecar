#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
	    
//CAN1��ʼ��
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;	
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1;//�ȼ�ȥ1.����������
	tbs2-=1;
	tbs1-=1;
	brp-=1;

//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB8 PB9
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB8����ΪCAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB9����ΪCAN1

	CAN1->MCR=0x0000;	//�˳�˯��ģʽ(ͬʱ��������λΪ0)
	CAN1->MCR|=1<<0;		//����CAN1�����ʼ��ģʽ
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2;//�����ʼ��ģʽʧ��
	}
	CAN1->MCR|=0<<7;		//��ʱ�䴥��ͨ��ģʽ
	CAN1->MCR|=0<<6;		//����Զ����߹���
	CAN1->MCR|=0<<5;		//˯��ģʽͨ���������(���CAN1->MCR��SLEEPλ)
	CAN1->MCR|=1<<4;		//��ֹ�����Զ�����
	CAN1->MCR|=0<<3;		//���Ĳ�����,�µĸ��Ǿɵ�
	CAN1->MCR|=0<<2;		//���ȼ��ɱ��ı�ʶ������
	CAN1->BTR=0x00000000;	//���ԭ��������.
	CAN1->BTR|=mode<<30;	//ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
	CAN1->BTR|=tsjw<<24; 	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ
	CAN1->BTR|=tbs2<<20; 	//Tbs2=tbs2+1��ʱ�䵥λ
	CAN1->BTR|=tbs1<<16;	//Tbs1=tbs1+1��ʱ�䵥λ
	CAN1->BTR|=brp<<0;  	//��Ƶϵ��(Fdiv)Ϊbrp+1
						//������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->MCR&=~(1<<0);	//����CAN1�˳���ʼ��ģʽ
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3;//�˳���ʼ��ģʽʧ��
	}
	//��������ʼ��
	CAN1->FMR|=1<<0;			//�������鹤���ڳ�ʼ��ģʽ
	CAN1->FA1R&=~(1<<0);		//������0������
	CAN1->FS1R|=1<<0; 		//������λ��Ϊ32λ.
	CAN1->FM1R|=0<<0;		//������0�����ڱ�ʶ������λģʽ
	CAN1->FFA1R|=0<<0;		//������0������FIFO0
	CAN1->sFilterRegister[0].FR1=0X00000000;//32λID
	CAN1->sFilterRegister[0].FR2=0X00000000;//32λMASK
	CAN1->FA1R|=1<<0;		//���������0
	CAN1->FMR&=0<<0;			//���������������ģʽ

#if CAN1_RX0_INT_ENABLE
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}  
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
//id:��׼ID(11λ)/��չID(11λ+18λ)	    
//ide:0,��׼֡;1,��չ֡
//rtr:0,����֡;1,Զ��֡
//len:Ҫ���͵����ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
//*dat:����ָ��.
//����ֵ:0~3,������.0XFF,����Ч����.
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;			//����0Ϊ��
	else if(CAN1->TSR&(1<<27))mbox=1;	//����1Ϊ��
	else if(CAN1->TSR&(1<<28))mbox=2;	//����2Ϊ��
	else return 0XFF;					//�޿�����,�޷����� 
	CAN1->sTxMailBox[mbox].TIR=0;		//���֮ǰ������
	if(ide==0)	//��׼֡
	{
		id&=0x7ff;//ȡ��11λstdid
		id<<=21;		  
	}else		//��չ֡
	{
		id&=0X1FFFFFFF;//ȡ��32λextid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F;//�õ�����λ
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;		   //����DLC.
	//���������ݴ�������.
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //��������������
	return mbox;
}
//��÷���״̬.
//mbox:������;
//����ֵ:����״̬. 0,����;0X05,����ʧ��;0X07,���ͳɹ�.
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			//RQCP0
			sta |= CAN1->TSR&(1<<1);			//TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24);	//TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		//RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		//TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25);	//TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	//RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	//TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26);	//TME2
			break;
		default:
			sta=0X05;//����Ų���,�϶�ʧ��.
		break;
	}
	return sta;
} 
//�õ���FIFO0/FIFO1�н��յ��ı��ĸ���.
//fifox:0/1.FIFO���;
//����ֵ:FIFO0/FIFO1�еı��ĸ���.
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN1->RF0R&0x03; 
	else if(fifox==1)return CAN1->RF1R&0x03; 
	else return 0;
}
//��������
//fifox:�����
//id:��׼ID(11λ)/��չID(11λ+18λ)	    
//ide:0,��׼֡;1,��չ֡
//rtr:0,����֡;1,Զ��֡
//len:���յ������ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
//dat:���ݻ�����
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//�õ���ʶ��ѡ��λ��ֵ  
 	if(*ide==0)//��׼��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	   //��չ��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//�õ�Զ�̷�������ֵ.
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;//�õ�DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//�õ�FMI
	//��������
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  	if(fifox==0)CAN1->RF0R|=0X20;//�ͷ�FIFO0����
	else if(fifox==1)CAN1->RF1R|=0X20;//�ͷ�FIFO1����	 
}

#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len,Can_Receive;     
	u8 temp_rxbuf[8];
 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
	if(id==0x121)
	{
  Can_Receive=temp_rxbuf[0];
	Remoter_Ch1=0,Remoter_Ch2=0; //���յ��ź�֮�� ���κ�ģң��
	if(Can_Receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
	else if(Can_Receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
	else if(Can_Receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
	else if(Can_Receive==0x42||Can_Receive==0x43||Can_Receive==0x44)	
												Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
	else if(Can_Receive==0x46||Can_Receive==0x47||Can_Receive==0x48)	    //��
												Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
	else Can_Receive=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
	
		
	if(Can_Receive==0x59)  Flag_sudu=2;  //���ٵ���Ĭ��ֵ��
	if(Can_Receive==0x58)  Flag_sudu=1;  //���ٵ� 
	}
}
#endif

//CAN1����һ������(�̶���ʽ:IDΪ0X601,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//�ȴ����ͽ���
	if(i>=0XFFF)return 1;							//����ʧ��?
	return 0;										//���ͳɹ�;
}
//CAN1�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;			//û�н��յ�����,ֱ���˳� 	 
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//��ȡ����
    if(id!=0x12||ide!=0||rtr!=0)len=0;		//���մ���	   
	return len;	
}


u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//�ȴ����ͽ���
	if(i>=0XFFF)return 1;							//����ʧ��?
	return 0;										//���ͳɹ�;
}

/* 
*  �������ƣ�CAN1_Send_Numm(u32 id,u8* msg)
*  ��ڲ�����id  ID�ţ�msg  ���������������
*  ��������� 1������ʧ�ܣ�   0�����ͳɹ�  
*  �������ܣ���������id����һ�����������
*/

u8 CAN1_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//�ȴ����ͽ���
	if(i>=0XFFF)return 1;							//����ʧ��?
	return 0;
}
 
void CAN1_SEND(void) 
{
//        u8 Direction_A,Direction_B,Direction_C,Direction_D;
//	      u16 Temp_Pitch,Temp_Roll,Temp_Yaw,Temp_GZ;
//	     static u8 Flag_Send;
//	           if(Encoder_A>0) Direction_A=0;
//        else if(Encoder_A<0) Direction_A=2;
//	      else                 Direction_A=1;
//	
//		         if(Encoder_B>0) Direction_B=0;
//        else if(Encoder_B<0) Direction_B=2;
//	      else                 Direction_B=1;     
//	
//		         if(Encoder_C>0) Direction_C=0;
//        else if(Encoder_C<0) Direction_C=2;
//	      else                 Direction_C=1;
//	
//			       if(Encoder_D>0) Direction_D=0;
//        else if(Encoder_D<0) Direction_D=2;
//	      else                 Direction_D=1;
//	
//	      Temp_Pitch=Pitch*100+30000;
//	      Temp_Roll=  Roll*100+30000;
//	      Temp_Yaw=    Yaw*100+30000;
//      	Temp_GZ=gyro[2]+32768;
//	      Flag_Send=!Flag_Send;
//        if(Flag_Send==0)//�����η���ȫ�������ݣ���һ��
//				{	
//			  txbuf[0]=Temp_Pitch>>8;
//				txbuf[1]=Temp_Pitch&0x00ff;		
//				txbuf[2]=Temp_Roll>>8;
//				txbuf[3]=Temp_Roll&0x00ff;
//				txbuf[4]=Temp_Yaw>>8;
//				txbuf[5]=Temp_Yaw&0x00ff;
//				txbuf[6]=Temp_GZ>>8;	
//			  txbuf[7]=Temp_GZ&0x00ff;	
//				CAN1_Send_Num(0x100,txbuf);	
//				}
//				else//�ڶ���
//				{	
//        txbuf[0]=abs(Encoder_A);  
//				txbuf[1]=Direction_A; 
//				txbuf[2]=abs(Encoder_B);  
//				txbuf[3]=Direction_B;
//				txbuf[4]=abs(Encoder_C);
//				txbuf[5]=Direction_C;
//				txbuf[6]=abs(Encoder_D);	
//				txbuf[7]=Direction_D;			
//      	CAN1_Send_Num(0x101,txbuf);					
//	      }
}



