/**
  ******************************************************************************
  * @file    enet.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.6.23
  * @brief   超核K60�ƺ件�?以太�?驱动文件
  ******************************************************************************
  */
#include "enet2.h"

#ifdef DEBUG_PRINT
#include "uart.h"
#endif

//��⹉以太网DMA��솲�?
static  uint8_t xENETTxDescriptors_unaligned[ ( 1 * sizeof( NBUF ) ) + 16 ];
static  uint8_t pxENETRxDescriptors_unaligned[ ( CFG_NUM_ENET_RX_BUFFERS * sizeof( NBUF ) ) + 16 ];
static NBUF *pxENETTxDescriptor;
static NBUF *pxENETRxDescriptors;

//以太网接收缓冲区
static uint8_t ucENETRxBuffers[ ( CFG_NUM_ENET_RX_BUFFERS * CFG_ENET_BUFFER_SIZE ) + 16 ];
static uint32_t uxNextRxBuffer = 0;

/***********************************************************************************************
 �ɟ能��⻥太网��솲区初��Ɍ�
 形参�?
 返回�?
 详解��⻥太网模块��뵱� ��솲�?描述�?类似USB) 来管理以太网
************************************************************************************************/
static void ENET_BDInit(void)
{
  unsigned long ux;
	unsigned char *pcBufPointer;
	//寻���16字节对��空间
	pcBufPointer = &( xENETTxDescriptors_unaligned[ 0 ] );
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETTxDescriptor = ( NBUF * ) pcBufPointer;	
	//寻���16字节对��空间
	pcBufPointer = &( pxENETRxDescriptors_unaligned[ 0 ] );
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETRxDescriptors = ( NBUF * ) pcBufPointer;

	pxENETTxDescriptor->length = 0;
	pxENETTxDescriptor->status = 0;
	pxENETTxDescriptor->ebd_status = TX_BD_IINS | TX_BD_PINS;
	//寻���16字节对��空间
	pcBufPointer = &( ucENETRxBuffers[ 0 ] );
	while((( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	//初始化接收描述符
	for( ux = 0; ux < CFG_NUM_ENET_RX_BUFFERS; ux++ )
	{
	    pxENETRxDescriptors[ ux ].status = RX_BD_E;
	    pxENETRxDescriptors[ ux ].length = 0;
      pxENETRxDescriptors[ ux ].data = (uint8_t *)__REV((uint32_t)pcBufPointer);
	    pcBufPointer += CFG_ENET_BUFFER_SIZE;
	    pxENETRxDescriptors[ ux ].bdu = 0x00000000;
	    pxENETRxDescriptors[ ux ].ebd_status = RX_BD_INT;
	}
	//���后一个描述符设置为Warp
	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
	//�?描述符开�?
	uxNextRxBuffer = 0;
	
}
/***********************************************************************************************
 �ɟ能�뵮�算MAC地址
 形参�?
 返回�?
 详解�?
************************************************************************************************/
uint8_t ENET_HashAddress(const uint8_t* addr)
{
  uint32_t crc;
  uint8_t byte;
  int i, j;
  crc = 0xFFFFFFFF;
  for(i=0; i<6; ++i)
  {
    byte = addr[i];
    for(j=0; j<8; ++j)
    {
      if((byte & 0x01)^(crc & 0x01))
      {
        crc >>= 1;
        crc = crc ^ 0xEDB88320;
      }
      else
        crc >>= 1;
      byte >>= 1;
    }
  }
  return (uint8_t)(crc >> 26);
}
/***********************************************************************************************
 �ɟ能�뵮�置MAC地址
 形参�?
 返回�?
 详解�?
************************************************************************************************/
void ENET_SetAddress(const uint8_t *pa)
{
  uint8_t crc;
  //设置物理地址
  ENET->PALR = (uint32_t)((pa[0]<<24) | (pa[1]<<16) | (pa[2]<<8) | pa[3]);
  ENET->PAUR = (uint32_t)((pa[4]<<24) | (pa[5]<<16));
  
  //�ݹ据物理地址计算并设置独��ɜ�址�����寄存器的��?
  crc = ENET_HashAddress(pa);
  if(crc >= 32)
    ENET->IAUR |= (uint32_t)(1 << (crc - 32));
  else
    ENET->IALR |= (uint32_t)(1 << crc);
}
/***********************************************************************************************
 �ɟ能�번���Ɍ�以太网模�?
 形参�?
 返回�?成功 1失败
 详解�?
************************************************************************************************/
uint8_t ENET_Init(ENET_InitTypeDef* ENET_InitStrut)
{
	uint16_t usData;
	uint16_t timeout = 0;
	//初始化结构体
	//enetdev.recflag = 0;
	//enetdev.linkstate = LINK_STATE_OFF; 
  //使能ENET�߶钟
  SIM->SCGC2 |= SIM_SCGC2_ENET_MASK;
  //允许并发访问MPU控制�?
  MPU->CESR = 0;         
  //��솲区描述符初始�?
  ENET_BDInit();
	//开PORT�߶钟
	SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
	//很��覲��ɡ�?
	MCG->C2 &= ~MCG_C2_EREFS_MASK;
	//�ո��以太�?
	ENET->ECR = ENET_ECR_RESET_MASK;
	for( usData = 0; usData < 100; usData++ )
	{
		//__NOP;
	}
  //初始化MII接口
  ENET_MiiInit();  

	//开启中�?
	NVIC_EnableIRQ(ENET_Transmit_IRQn);
	NVIC_EnableIRQ(ENET_Receive_IRQn);
	NVIC_EnableIRQ(ENET_Error_IRQn);
  //使能GPIO引脚�᫔��ɟ能
  PORTB->PCR[0]  = PORT_PCR_MUX(4); 
  PORTB->PCR[1]  = PORT_PCR_MUX(4); 
	PORTA->PCR[12] =  PORT_PCR_MUX(4);  
	PORTA->PCR[13] =  PORT_PCR_MUX(4);  
	PORTA->PCR[14] =  PORT_PCR_MUX(4);  
	PORTA->PCR[15] =  PORT_PCR_MUX(4);  
	PORTA->PCR[16] =  PORT_PCR_MUX(4);  
	PORTA->PCR[17] =  PORT_PCR_MUX(4);  
  //等待PHY收发器复位���?
  do
  {
    DelayMs(10);
		timeout++;
		if(timeout > 500) return 1;
    usData = 0xffff;
    ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
        
  } while( usData == 0xffff );

#ifdef DEBUG_PRINT
  UART_printf("PHY_PHYIDR1=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYIDR2, &usData );
  UART_printf("PHY_PHYIDR2=0x%X\r\n",usData); 
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_ANLPAR, &usData );
  UART_printf("PHY_ANLPAR=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_ANLPARNP, &usData );
  UART_printf("PHY_ANLPARNP=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_PHYSTS, &usData );
  UART_printf("PHY_PHYSTS=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_MICR, &usData );
  UART_printf("PHY_MICR=0x%X\r\n",usData);
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_MISR, &usData );
  UART_printf("PHY_MISR=0x%X\r\n",usData);
#endif 
  //开始自�ɨ���?
  ENET_MiiWrite(CFG_PHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );

#ifdef DEBUG_PRINT
  ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMCR, &usData );
  UART_printf("PHY_BMCR=0x%X\r\n",usData);
#endif 
  //等待���动协商完成
  do
  {
    DelayMs(10);
		timeout++;
		if(timeout > 500) return 1;
    ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMSR, &usData );

  } while( !( usData & PHY_BMSR_AN_COMPLETE ) );
  //�ݹ据协商结果设置ENET模块
	usData = 0;
	ENET_MiiRead(CFG_PHY_ADDRESS, PHY_STATUS, &usData );	
  
  //清除卿���和组地址�����寄存�?
  ENET->IALR = 0;
  ENET->IAUR = 0;
  ENET->GALR = 0;
  ENET->GAUR = 0;
  //设置ENET模块MAC地址
  ENET_SetAddress(ENET_InitStrut->pMacAddress);
  //设置接收控制寄存器，���大���度、RMII模��、接收CRC�ݡ验�?
  ENET->RCR = ENET_RCR_MAX_FL(CFG_ENET_MAX_PACKET_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK | ENET_RCR_RMII_MODE_MASK;

  //清除发送接收���?
  ENET->TCR = 0;
  //��뵮�方��设置
  if( usData & PHY_DUPLEX_STATUS )
  {
    //全双�?
    ENET->RCR &= (unsigned long)~ENET_RCR_DRT_MASK;
    ENET->TCR |= ENET_TCR_FDEN_MASK;
		#ifdef DEBUG_PRINT
			UART_printf("全双工\r\n");
		#endif 
  }
  else
  {
    //半双�?
    ENET->RCR |= ENET_RCR_DRT_MASK;
    ENET->TCR &= (unsigned long)~ENET_TCR_FDEN_MASK;
		#ifdef DEBUG_PRINT
		UART_printf("半双工\r\n");
		#endif 
  }
  //���⿡�͟率设置
  if( usData & PHY_SPEED_STATUS )
  {
    //10Mbps
    ENET->RCR |= ENET_RCR_RMII_10T_MASK;
  }

  //使用增强型缓冲区描述�?
  ENET->ECR = ENET_ECR_EN1588_MASK;

	//设置接收��솲区大�?
	ENET->MRBR = ENET_MRBR_R_BUF_SIZE(CFG_ENET_MAX_PACKET_SIZE);

	//指向环形��솲区描述符�Є首地址(RX)
	ENET->RDSR = (uint32_t)  pxENETRxDescriptors;

	//指向环形��솲区描述符�Є首地址(TX)
	ENET->TDSR = (uint32_t) pxENETTxDescriptor;

	//清楚�؀���中断标�?
	ENET->EIR = ( uint32_t ) 0xFFFFFFFF;

	//使能中断
	ENET->EIMR = ENET_EIR_TXF_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_RXB_MASK | ENET_EIMR_UN_MASK | ENET_EIMR_RL_MASK | ENET_EIMR_LC_MASK | ENET_EIMR_BABT_MASK | ENET_EIMR_BABR_MASK | ENET_EIMR_EBERR_MASK;

	//使能MAC模块
	ENET->ECR |= ENET_ECR_ETHEREN_MASK;
  //表明接收��솲区为�?
	ENET->RDAR = ENET_RDAR_RDAR_MASK;
	//检�ҥ连接状��?
//	enetdev.linkstate =  ENET_MiiLinkState();
	return 0;
}
/***********************************************************************************************
 �ɟ能：配置物理层
 形参�?
 返回�?
 详解�?
************************************************************************************************/
void ENET_MiiInit(void)
{
	uint8_t i;
	GetCPUInfo();
	i = (CPUInfo.BusClock/1000)/1000;
  ENET->MSCR = 0 | ENET_MSCR_MII_SPEED((2*i/5)+1);
}

//物理层写入数�?
uint8_t ENET_MiiWrite(uint16_t phy_addr, uint16_t reg_addr, uint16_t data)
{
	uint32_t timeout;
  //清除MII中断事件
	ENET->EIR = ENET_EIR_MII_MASK;
  //初始化MII管理帧寄��뙨
	ENET->MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x01)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02)
            | ENET_MMFR_DATA(data);
  //等待MII传输完成中断事件
  for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
  {
    if (ENET->EIR & ENET_EIR_MII_MASK)
      break;
  }
  if(timeout == MII_TIMEOUT) 
    return 1;
  //清除MII中断事件
  ENET->EIR = ENET_EIR_MII_MASK;
  return 0;
}
/***********************************************************************************************
 �ɟ能：RMII�?读取数据
 形参：phy_addr: 接口地址  reg_addr:要读取的寄存�? *data:数据
 返回�? 成功  1失败
 详解�?
************************************************************************************************/
uint8_t ENET_MiiRead(uint16_t phy_addr, uint16_t reg_addr, uint16_t *data)
{
	uint32_t timeout;
	//清除MII中断事件
	ENET->EIR = ENET_EIR_MII_MASK;
	//初始化MII管理帧寄��뙨
  ENET->MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x2)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02);
  
	//等待MII传输完成中断事件
	for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
  {
    if (ENET->EIR & ENET_EIR_MII_MASK)
      break;
  }
  if(timeout == MII_TIMEOUT) 
    return 1;
  //清除MII中断事件
  ENET->EIR = ENET_EIR_MII_MASK;
  *data = ENET->MMFR & 0x0000FFFF;
  return 0;
}
/***********************************************************************************************
 �ɟ能：查看网线连接状��?
 形参�?
 返回：ENET_PHY_LINK_STATE
					LINK_STATE_ON,
					LINK_STATE_OFF,
 详解�?
************************************************************************************************/
uint8_t ENET_MiiLinkState(void)
{
	uint16_t reg = 0;
	ENET_MiiRead(CFG_PHY_ADDRESS, PHY_BMSR, &reg );
	if(reg & 0x0004)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/***********************************************************************************************
 �ɟ能�벏��́一个以太帧 
 形参�?ch:数据指针   len:长度
 返回�?
 详解�?
************************************************************************************************/
void ENET_MacSendData(uint8_t *ch, uint16_t len)
{
  //检�ҥ当前发�́缓冲区描述符是否可�?
	while( pxENETTxDescriptor->status & TX_BD_R )
	{
		
	}
  //设置发送缓冲区描述�?
  pxENETTxDescriptor->data = (uint8_t *)__REV((uint32_t)ch);		
  pxENETTxDescriptor->length = __REVSH(len);
	pxENETTxDescriptor->bdu = 0x00000000;
	pxENETTxDescriptor->ebd_status = TX_BD_INT | TX_BD_TS;// | TX_BD_IINS | TX_BD_PINS;
	pxENETTxDescriptor->status = ( TX_BD_R | TX_BD_L | TX_BD_TC | TX_BD_W );
  //使能发�?
  ENET->TDAR = ENET_TDAR_TDAR_MASK;
}

/***********************************************************************************************
 �ɟ能：接收一个以太帧
 形参�?ch:数据指针 
 返回�벸�长度
 详解�?
************************************************************************************************/
uint16_t ENET_MacRecData(uint8_t *ch)
{
	uint16_t len = 0;
	ch = ch;
	//寻����Ǟ空�Є缓冲区描述�?
	if((pxENETRxDescriptors[uxNextRxBuffer].status & RX_BD_E ) == 0)
	{
		//读取数据
		len =  __REVSH(pxENETRxDescriptors[ uxNextRxBuffer ].length);
		memcpy(ch,(uint8_t *)__REV((uint32_t)pxENETRxDescriptors[ uxNextRxBuffer ].data),len);
		//表示已经读取改缓冲区数据
		pxENETRxDescriptors[uxNextRxBuffer].status |= RX_BD_E;
		uxNextRxBuffer++;
		if( uxNextRxBuffer >= CFG_NUM_ENET_RX_BUFFERS )
		{
			uxNextRxBuffer = 0;
		}
		ENET->RDAR = ENET_RDAR_RDAR_MASK;
	}
	return len;
	
}
/***********************************************************************************************
 �ɟ能��⻥太网发送��成中�?
 形参�?
 返回�?
 详解�?
************************************************************************************************/
void ENET_Transmit_IRQHandler(void)
{
	ENET->EIR |= ENET_EIMR_TXF_MASK; 
}
/***********************************************************************************************
 �ɟ能��⻥太网接收中断
 形参�?
 返回�?
 详解�?
************************************************************************************************/
uint8_t gEnetFlag = 0;
#if 0
void ENET_Receive_IRQHandler(void)
{
	ENET->EIR |= ENET_EIMR_RXF_MASK; 
	gEnetFlag = 1;
}
#endif

void ENET_Error_IRQHandler(void)
{
	//UART_printf("以太网错误\r\n");
}












#define _MAC1          0x1E

//     <o>Address byte 2 <0x00-0xff>
//     <i> Default: 0x30
#define _MAC2          0x30

//     <o>Address byte 3 <0x00-0xff>
//     <i> Default: 0x6C
#define _MAC3          0x6C

//     <o>Address byte 4 <0x00-0xff>
//     <i> Default: 0x00
#define _MAC4          0xA2

//     <o>Address byte 5 <0x00-0xff>
//     <i> Default: 0x00
#define _MAC5          0x45

//     <o>Address byte 6 <0x00-0xff>
//     <i> Default: 0x01
#define _MAC6          0x5E
/* Definitions */
#define ETH_ADRLEN      6         /* Ethernet Address Length in bytes        */
#define IP_ADRLEN       4         /* IP Address Length in bytes              */
#define OS_HEADER_LEN   4         /* TCPnet 'os_frame' header size           */
                                  /* Frame Header length common for all      */
#define PHY_HEADER_LEN  (2*ETH_ADRLEN + 2) /* network interfaces.            */
#define ETH_MTU         1514      /* Ethernet Frame Max Transfer Unit        */
#define PPP_PROT_IP     0x0021    /* PPP Protocol type: IP                   */
#define TCP_DEF_WINSIZE 4380      /* TCP default window size                 */
#define PASSW_SZ        20        /* Authentication Password Buffer size     */
 U8     own_hw_adr[ETH_ADRLEN] = {_MAC1,_MAC2,_MAC3,_MAC4,_MAC5,_MAC6};

/* Local variables */
static U8 TxBufIndex;
static U8 RxBufIndex;

/* ENET local DMA Descriptors. */
static __align(16) RX_Desc Rx_Desc[NUM_RX_BUF];
static __align(16) TX_Desc Tx_Desc[NUM_TX_BUF];

/* ENET local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

#define IDX(x) (x/2)
#define SWE(n) ((((n) & 0x00FF) << 8) | (((n) & 0xFF00) >> 8))


/* Local Function Prototypes */
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);


/*-----------------------------------------------------------------------------
 *      ENET Ethernet Driver Functions
 *-----------------------------------------------------------------------------
 *  Required functions for Ethernet driver module:
 *  a. Polling mode: - void init_ethernet ()
 *                   - void send_frame (OS_FRAME *frame)
 *                   - void poll_ethernet (void)
 *  b. Interrupt mode: - void init_ethernet ()
 *                     - void send_frame (OS_FRAME *frame)
 *                     - void int_enable_eth ()
 *                     - void int_disable_eth ()
 *                     - interrupt function 
 *----------------------------------------------------------------------------*/

/* Local Function Prototypes */
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);




/*--------------------------- init_ethernet ----------------------------------*/
void init_ethernet (void) {
  /* Initialize the ETH ethernet controller. */
  U32 regv,tout,id1,id2,ctrl;
  
  OSC->CR   |= OSC_CR_ERCLKEN_MASK;    /* Enable external reference clock    */
  SIM->SCGC2 |= SIM_SCGC2_ENET_MASK;    /* Enable ENET module gate clocking   */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK |  /* Enable Port A module gate clocking */
                SIM_SCGC5_PORTB_MASK ;  /* Enable Port B module gate clocking */

  /* Configure Ethernet Pins  */
  PORTA->PCR[5]  &= ~(PORT_PCR_MUX_MASK | PORT_PCR_PS_MASK);
  PORTA->PCR[5]  |=  PORT_PCR_MUX(4);   /* Pull-down on RX_ER is enabled      */

  PORTA->PCR[12]  = (PORTA->PCR[12] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[13]  = (PORTA->PCR[13] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[14]  = (PORTA->PCR[14] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[15]  = (PORTA->PCR[15] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[16]  = (PORTA->PCR[16] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[17]  = (PORTA->PCR[17] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);

  PORTB->PCR[0]   = (PORTB->PCR[0]  & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4)| PORT_PCR_ODE_MASK;
  PORTB->PCR[1]   = (PORTB->PCR[1]  & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);

  /* Reset Ethernet MAC */
  ENET->ECR =  ENET_ECR_RESET_MASK;
  while (ENET->ECR & ENET_ECR_RESET_MASK);

  /* Set MDC clock frequency @ 50MHz MAC clock frequency */
  ENET->MSCR = ENET_MSCR_MII_SPEED(0x13);

  /* Set receive control */
  ENET->RCR = ENET_RCR_MAX_FL (0x5EE) |
              ENET_RCR_RMII_MODE_MASK | /* MAC Configured for RMII operation  */
              ENET_RCR_MII_MODE_MASK  ; /* This bit must always be set        */

  /* Set transmit control */
  ENET->TCR = ENET_TCR_ADDINS_MASK |    /* MAC overwrites the source MAC */
              ENET_TCR_ADDSEL(0)   ;    /* MAC address is in PADDR 1 and 2 */

  ENET->MRBR = ETH_BUF_SIZE;

  /* Set thresholds */
  ENET->RAEM = 4;
  ENET->RAFL = 4;
  ENET->TAEM = 4;
  ENET->TAFL = 8;
  ENET->RSFL = 0;                       /* Store and forward on the RX FIFO   */
  ENET->FTRL = 0x7FF;                   /* Frame Truncation Length            */

  /* Read PHY ID */
  id1 = read_PHY (PHY_REG_ID1);
  id2 = read_PHY (PHY_REG_ID2);

  /* Check if this is a KSZ8041NL PHY. */
  if (((id1 << 16) | (id2 & 0xFFF0)) == PHY_ID_KSZ8041) {
    /* Put the PHY in reset mode */
    write_PHY (PHY_REG_BCTRL, 0x8000);

    /* Wait for hardware reset to end. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BCTRL);
      if (!(regv & 0x8800)) {
        /* Reset complete, device not Power Down. */
        break;
      }
    }
    /* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_BCTRL, PHY_FULLD_10M);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_BCTRL, PHY_FULLD_100M);
#else
    /* Use autonegotiation about the link speed. */
    write_PHY (PHY_REG_BCTRL, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BSTAT);
      if (regv & 0x0020) {        
        break;                          /* Autonegotiation Complete           */
      }
    }
#endif
    /* Check the link status. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BSTAT);
      if (regv & 0x0004) {        
        break;                          /* Link is on                         */
      }
    }

    /* Check Operation Mode Indication in PHY Control register */ 
    regv = read_PHY (PHY_REG_PC2);
    /* Configure Full/Half Duplex mode. */
    switch ((regv & 0x001C) >> 2) {
      case 1:  ctrl = PHY_CON_10M  | PHY_CON_HD; break;
      case 2:  ctrl = PHY_CON_100M | PHY_CON_HD; break;
      case 5:  ctrl = PHY_CON_10M  | PHY_CON_FD; break;
      case 6:  ctrl = PHY_CON_100M | PHY_CON_FD; break;
      default: ctrl = 0;                         break;
    }
    if (ctrl & PHY_CON_FD) {
      ENET->TCR |= ENET_TCR_FDEN_MASK;  /* Enable Full duplex                 */
    }

    /* Configure 100MBit/10MBit mode. */
    if (ctrl & PHY_CON_100M) {      
      ENET->RCR &= ~ENET_RCR_RMII_10T_MASK; /* 100MBit mode.                  */
    }
  }

  /* MAC address filtering, accept multicast packets. */

  /* Set the Ethernet MAC Address registers */
  ENET->PAUR = ENET_PAUR_TYPE (0x8808) |
               ENET_PAUR_PADDR2(((U32)own_hw_adr[4] << 8) | (U32)own_hw_adr[5]);
  ENET->PALR = ((U32)own_hw_adr[0] << 24) | (U32)own_hw_adr[1] << 16 |
               ((U32)own_hw_adr[2] <<  8) | (U32)own_hw_adr[3];

  ENET->MIBC |= ENET_MIBC_MIB_CLEAR_MASK;

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Reset all interrupts */
  ENET->EIR = 0x7FFF8000;
  
  /* Enable receive interrupt */
  ENET->EIMR = ENET_EIMR_RXF_MASK;

  /* Enable Ethernet, reception and transmission are possible */
  ENET->ECR  |= ENET_ECR_EN1588_MASK | ENET_ECR_ETHEREN_MASK;

  /* Start MAC receive descriptor ring pooling */
  ENET->RDAR = ENET_RDAR_RDAR_MASK;
}
/*--------------------------- int_enable_eth ---------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ (ENET_Receive_IRQn);
}


/*--------------------------- int_disable_eth --------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ (ENET_Receive_IRQn);
}


/*--------------------------- send_frame -------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to ETH ethernet controller */
  U32 *sp,*dp;
  U32 i,j, adr;

  j = TxBufIndex;
  /* Wait until previous packet transmitted. */
  while (Tx_Desc[j].TBD[IDX(0)] & SWE(DESC_TX_R));

  adr  = SWE(Tx_Desc[j].TBD[IDX(4)]) << 16;
  adr |= SWE(Tx_Desc[j].TBD[IDX(6)]);

  dp = (U32 *)(adr & ~3);
  sp = (U32 *)&frame->data[0];

  /* Copy frame data to ETH IO buffer. */
  for (i = (frame->length + 3) >> 2; i; i--) {
    *dp++ = *sp++;
  }
  Tx_Desc[j].TBD[IDX(2)]  = SWE(frame->length);
  Tx_Desc[j].TBD[IDX(0)] |= SWE(DESC_TX_R | DESC_TX_L | DESC_TX_TC);
  Tx_Desc[j].TBD[IDX(0x10)] = 0;
  if (++j == NUM_TX_BUF) j = 0;
  TxBufIndex = j;
  /* Start frame transmission. */
  ENET->TDAR = ENET_TDAR_TDAR_MASK;
}

/*--------------------------- interrupt_ethernet -----------------------------*/

void ENET_Receive_IRQHandler (void) {
  OS_FRAME *frame;
  U32 i, adr, RxLen;
  U32 *sp,*dp;

  i = RxBufIndex;
  do {
    if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_TR)) {
      /* Frame is truncated */
      goto rel;
    }
    RxLen = SWE(Rx_Desc[i].RBD[IDX(2)]);
    
    if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_L)) {
      /* Last in a frame, length includes CRC bytes */
      RxLen -= 4;
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_OV)) {
        /* Receive FIFO overrun */
        goto rel;
      }
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_NO)) {
        /* Received non-octet aligned frame */
        goto rel;
      }
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_CR)) {
        /* CRC or frame error */
        goto rel;
      }
    }

    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }
    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = (OS_FRAME*)malloc (RxLen | 0x80000000);
    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      adr  = SWE(Rx_Desc[i].RBD[IDX(4)]) << 16;
      adr |= SWE(Rx_Desc[i].RBD[IDX(6)]);
      sp = (U32 *)(adr & ~3);
      dp = (U32 *)&frame->data[0];
      for (RxLen = (RxLen + 3) >> 2; RxLen; RxLen--) {
        *dp++ = *sp++;
      }
     // put_in_queue (frame);
    }
    /* Release this frame from ETH IO buffer. */
rel:Rx_Desc[i].RBD[IDX(0)] |= SWE(DESC_RX_E);
    Rx_Desc[i].RBD[IDX(0x10)] = 0;

    if (++i == NUM_RX_BUF) i = 0;
    RxBufIndex = i;
  }
  while (!(Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_E)));
  RxBufIndex = i;

  /* Clear pending interrupt bits */
  ENET->EIR = ENET_EIR_RXB_MASK | ENET_EIR_RXF_MASK;
  
  /* Start MAC receive descriptor ring pooling */
  ENET->RDAR = ENET_RDAR_RDAR_MASK;
}


/*--------------------------- rx_descr_init ----------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive DMA Descriptor array. */
  U32 i,next;

  RxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_RX_BUF; i++) {
    if (++next == NUM_RX_BUF) next = 0;
    Rx_Desc[i].RBD[IDX(0x00)] = SWE(DESC_RX_E);
    Rx_Desc[i].RBD[IDX(0x04)] = SWE((U32)&rx_buf[i] >> 16);
    Rx_Desc[i].RBD[IDX(0x06)] = SWE((U32)&rx_buf[i] & 0xFFFF);
    Rx_Desc[i].RBD[IDX(0x08)] = SWE(DESC_RX_INT);
    Rx_Desc[i].RBD[IDX(0x10)] = 0;
  }
  Rx_Desc[i-1].RBD[IDX(0)] |= SWE(DESC_RX_W);
  ENET->RDSR = (U32)&Rx_Desc[0];
}



/*--------------------------- tx_descr_init ----------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit DMA Descriptor array. */
  U32 i,next;

  TxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_TX_BUF; i++) {
    if (++next == NUM_TX_BUF) next = 0;
    Tx_Desc[i].TBD[IDX(0x00)] = SWE(DESC_TX_L);
    Tx_Desc[i].TBD[IDX(0x04)] = SWE((U32)&tx_buf[i] >> 16);
    Tx_Desc[i].TBD[IDX(0x06)] = SWE((U32)&tx_buf[i] & 0xFFFF);
    Tx_Desc[i].TBD[IDX(0x10)] = 0;
  }
  Tx_Desc[i-1].TBD[IDX(0x00)] |= SWE(DESC_TX_W);
  ENET->TDSR = (U32)&Tx_Desc[0];
}


/*--------------------------- write_PHY --------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  /* Clear MII Interrupt */
  ENET->EIR = ENET_EIR_MII_MASK;

  /* Send MDIO write command */
  ENET->MMFR =  ENET_MMFR_ST (1)            |
                ENET_MMFR_OP (1)            |
                ENET_MMFR_PA (PHY_DEF_ADDR) |
                ENET_MMFR_RA (PhyReg)       |
                ENET_MMFR_TA (2)            |
                ENET_MMFR_DATA (Value)      ;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if (ENET->EIR & ENET_EIR_MII_MASK) {
      break;
    }
  }
}


/*--------------------------- read_PHY ---------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  /* Clear MII Interrupt */
  ENET->EIR = ENET_EIR_MII_MASK;

  /* Send MDIO read command */
  ENET->MMFR =  ENET_MMFR_ST (1)            |
                ENET_MMFR_OP (2)            |
                ENET_MMFR_PA (PHY_DEF_ADDR) |
                ENET_MMFR_RA (PhyReg)       |
                ENET_MMFR_TA (2)            ;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if (ENET->EIR & ENET_EIR_MII_MASK) {
      break;
    }
  }
  return (ENET->MMFR & ENET_MMFR_DATA_MASK);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

