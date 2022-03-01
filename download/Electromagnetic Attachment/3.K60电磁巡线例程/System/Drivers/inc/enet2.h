/**
  ******************************************************************************
  * @file    enet.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.6.23
  * @brief   超核K60�ƺ件�?以太�?驱动文件
  ******************************************************************************
  */
#ifndef __ENET_H__
#define __ENET_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"

/* MII寄存器地址 */
#define PHY_BMCR                    (0x00)
#define PHY_BMSR                    (0x01)
#define PHY_PHYIDR1                 (0x02)
#define PHY_PHYIDR2                 (0x03)
#define PHY_ANAR                    (0x04)
#define PHY_ANLPAR                  (0x05)
#define PHY_ANLPARNP                (0x05)
#define PHY_ANER                    (0x06)
#define PHY_ANNPTR                  (0x07)
#define PHY_PHYSTS                  (0x10)
#define PHY_MICR                    (0x11)
#define PHY_MISR                    (0x12)
#define PHY_PAGESEL                 (0x13)


/* PHY_BMCR寄存器位��⹉ */
#define PHY_BMCR_RESET              (0x8000)
#define PHY_BMCR_LOOP               (0x4000)
#define PHY_BMCR_SPEED              (0x2000)
#define PHY_BMCR_AN_ENABLE          (0x1000)
#define PHY_BMCR_POWERDOWN          (0x0800)
#define PHY_BMCR_ISOLATE            (0x0400)
#define PHY_BMCR_AN_RESTART         (0x0200)
#define PHY_BMCR_FDX                (0x0100)
#define PHY_BMCR_COL_TEST           (0x0080)

/* PHY_BMSR寄存器位��⹉ */
#define PHY_BMSR_100BT4             (0x8000)
#define PHY_BMSR_100BTX_FDX         (0x4000)
#define PHY_BMSR_100BTX             (0x2000)
#define PHY_BMSR_10BT_FDX           (0x1000)
#define PHY_BMSR_10BT               (0x0800)
#define PHY_BMSR_NO_PREAMBLE        (0x0040)
#define PHY_BMSR_AN_COMPLETE        (0x0020)
#define PHY_BMSR_REMOTE_FAULT       (0x0010)
#define PHY_BMSR_AN_ABILITY         (0x0008)
#define PHY_BMSR_LINK               (0x0004)
#define PHY_BMSR_JABBER             (0x0002)
#define PHY_BMSR_EXTENDED           (0x0001)

/* PHY_ANAR寄存器位��⹉ */
#define PHY_ANAR_NEXT_PAGE          (0x8001)
#define PHY_ANAR_REM_FAULT          (0x2001)
#define PHY_ANAR_PAUSE              (0x0401)
#define PHY_ANAR_100BT4             (0x0201)
#define PHY_ANAR_100BTX_FDX         (0x0101)
#define PHY_ANAR_100BTX             (0x0081)
#define PHY_ANAR_10BT_FDX           (0x0041)
#define PHY_ANAR_10BT               (0x0021)
#define PHY_ANAR_802_3              (0x0001)

/* PHY_ANLPAR寄存器位��⹉ */
#define PHY_ANLPAR_NEXT_PAGE        (0x8000)
#define PHY_ANLPAR_ACK              (0x4000)
#define PHY_ANLPAR_REM_FAULT        (0x2000)
#define PHY_ANLPAR_PAUSE            (0x0400)
#define PHY_ANLPAR_100BT4           (0x0200)
#define PHY_ANLPAR_100BTX_FDX       (0x0100)
#define PHY_ANLPAR_100BTX           (0x0080)
#define PHY_ANLPAR_10BTX_FDX        (0x0040)
#define PHY_ANLPAR_10BT             (0x0020)


/* PHY_PHYSTS寄存器位��⹉ */
#define PHY_PHYSTS_MDIXMODE         (0x4000)
#define PHY_PHYSTS_RX_ERR_LATCH     (0x2000)
#define PHY_PHYSTS_POL_STATUS       (0x1000)
#define PHY_PHYSTS_FALSECARRSENSLAT (0x0800)
#define PHY_PHYSTS_SIGNALDETECT     (0x0400)
#define PHY_PHYSTS_PAGERECEIVED     (0x0100)
#define PHY_PHYSTS_MIIINTERRUPT     (0x0080)
#define PHY_PHYSTS_REMOTEFAULT      (0x0040)
#define PHY_PHYSTS_JABBERDETECT     (0x0020)
#define PHY_PHYSTS_AUTONEGCOMPLETE  (0x0010)
#define PHY_PHYSTS_LOOPBACKSTATUS   (0x0008)
#define PHY_PHYSTS_DUPLEXSTATUS     (0x0004)
#define PHY_PHYSTS_SPEEDSTATUS      (0x0002)
#define PHY_PHYSTS_LINKSTATUS       (0x0001)


/* PHY硬件特�?*/
#define PHY_STATUS								( 0x1F )
#define PHY_DUPLEX_STATUS							( 4<<2 )
#define PHY_SPEED_STATUS							( 1<<2 )
/* PHY收发器硬件地址 */
#define CFG_PHY_ADDRESS	            0x01


//Freescale处理器相关定�?

/* TX��솲区描述符位定�?*/
#define TX_BD_R			0x0080
#define TX_BD_TO1		0x0040
#define TX_BD_W			0x0020
#define TX_BD_TO2		0x0010
#define TX_BD_L			0x0008
#define TX_BD_TC		0x0004
#define TX_BD_ABC		0x0002

/* TX增强型缓冲区描述符位��⹉ */
#define TX_BD_INT       0x00000040 
#define TX_BD_TS        0x00000020 
#define TX_BD_PINS      0x00000010 
#define TX_BD_IINS      0x00000008 
#define TX_BD_TXE       0x00800000 
#define TX_BD_UE        0x00200000 
#define TX_BD_EE        0x00100000
#define TX_BD_FE        0x00080000 
#define TX_BD_LCE       0x00040000 
#define TX_BD_OE        0x00020000 
#define TX_BD_TSE       0x00010000 

#define TX_BD_BDU       0x00000080    

/* RX��솲区描述符位定�?*/
// 0���移�݇�֯ - �Ӷ�?大端�ݼ��
#define RX_BD_E			0x0080
#define RX_BD_R01		0x0040
#define RX_BD_W			0x0020
#define RX_BD_R02		0x0010
#define RX_BD_L			0x0008
#define RX_BD_M			0x0001
#define RX_BD_BC		0x8000
#define RX_BD_MC		0x4000
#define RX_BD_LG		0x2000
#define RX_BD_NO		0x1000
#define RX_BD_CR		0x0400
#define RX_BD_OV		0x0200
#define RX_BD_TR		0x0100

/* RX增强型缓冲区描述符位��⹉ */
#define RX_BD_ME               0x00000080    
#define RX_BD_PE               0x00000004    
#define RX_BD_CE               0x00000002    
#define RX_BD_UC               0x00000001
    
#define RX_BD_INT              0x00008000    

#define RX_BD_ICE              0x20000000    
#define RX_BD_PCR              0x10000000    
#define RX_BD_VLAN             0x04000000    
#define RX_BD_IPV6             0x02000000    
#define RX_BD_FRAG             0x01000000    

#define RX_BD_BDU              0x00000080   

/* MII接口超时 */
#define MII_TIMEOUT		0x1FFFF


typedef void (*ENET_ISR_CALLBACK)(void);

/* 以太帧相关定�?*/
#define CFG_NUM_ENET_TX_BUFFERS       1     //发送缓冲区个数
#define CFG_NUM_ENET_RX_BUFFERS	      8     //接收��솲区个�?
#define CFG_ENET_BUFFER_SIZE	      1520    //以太发送帧��솲区����?
#define CFG_ENET_MAX_PACKET_SIZE    1520    //以太发最大数据包长度

/* ��솲区描述符结构�?*/
  typedef struct
  {
  	uint16_t status;	            /* control and status */
  	uint16_t length;	            /* transfer length */
  	uint8_t  *data;	                /* buffer address */
  	uint32_t ebd_status;
  	uint16_t length_proto_type;
  	uint16_t payload_checksum;
  	uint32_t bdu;
  	uint32_t timestamp;
  	uint32_t reserverd_word1;
  	uint32_t reserverd_word2;
  } NBUF;


typedef struct
{
		uint8_t* pMacAddress;
}ENET_InitTypeDef;


//���构件实现的接口函数
uint8_t ENET_Init(ENET_InitTypeDef* ENET_InitStrut);
void ENET_MacSendData(uint8_t *ch, uint16_t len);
uint16_t ENET_MacRecData(uint8_t *ch);
uint8_t ENET_MiiWrite(uint16_t phy_addr, uint16_t reg_addr, uint16_t data);
uint8_t ENET_MiiRead(uint16_t phy_addr, uint16_t reg_addr, uint16_t *data);
void ENET_MiiInit(void);
uint8_t ENET_MiiLinkState(void);














/* ETH Memory Buffer configuration. */
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)         */
#define NUM_TX_BUF          2           /* 0x0600 for Tx (2*1536=3K)         */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size  */

/* uDMA Descriptors. */
typedef volatile struct {
  uint16_t RBD[16];
} RX_Desc;

typedef volatile struct {
  uint16_t TBD[16];
} TX_Desc;

/* Receive Buffer Descriptor Field Definitions */
#define DESC_RX_E           (1 << 15)   /* Empty                              */
#define DESC_RX_RO1         (1 << 14)   /* Receive software ownership         */
#define DESC_RX_W           (1 << 13)   /* Wrap                               */
#define DESC_RX_RO2         (1 << 12)   /* Receive software ownership         */
#define DESC_RX_L           (1 << 11)   /* Last in frame                      */
#define DESC_RX_M           (1 <<  8)   /* Miss                               */
#define DESC_RX_BC          (1 <<  7)   /* DA is broadcast                    */
#define DESC_RX_MC          (1 <<  6)   /* DA is multicast                    */
#define DESC_RX_LG          (1 <<  5)   /* Rx frame length violation          */
#define DESC_RX_NO          (1 <<  4)   /* Non-octet aligned frame            */
#define DESC_RX_CR          (1 <<  2)   /* Receive CRC or frame error         */
#define DESC_RX_OV          (1 <<  1)   /* Overrun                            */
#define DESC_RX_TR          (1 <<  0)   /* Receive frame is truncated         */

#define DESC_RX_INT         (1 <<  7)   /* Generate RXB/RXF interrupt         */
#define DESC_RX_BDU         (1 << 15)   /* Last descriptor update done        */

/* Transmit Buffer Descriptor Field Definitions */
#define DESC_TX_R           (1 << 15)   /* Ready                              */
#define DESC_TX_TO1         (1 << 14)   /* Transmit software ownership        */
#define DESC_TX_W           (1 << 13)   /* Wrap                               */
#define DESC_TX_TO2         (1 << 12)   /* Transmit software ownership        */
#define DESC_TX_L           (1 << 11)   /* Last in frame                      */
#define DESC_TX_TC          (1 << 10)   /* Transmit CRC                       */
#define DESC_TX_ABC         (1 <<  9)   /* Append bad CRC                     */

/* MII Management Time out values */
#define MII_WR_TOUT         0x00050000  /* MII Write timeout count            */
#define MII_RD_TOUT         0x00050000  /* MII Read timeout count             */

/* KSZ8041NL PHY Registers */
#define PHY_REG_BCTRL       0x00        /* Basic Control Register             */
#define PHY_REG_BSTAT       0x01        /* Basic Status Register              */
#define PHY_REG_ID1         0x02        /* PHY Identifier 1                   */
#define PHY_REG_ID2         0x03        /* PHY Identifier 2                   */
#define PHY_REG_ANA         0x04        /* Auto-Negotiation Advertisement     */
#define PHY_REG_ANLPA       0x05        /* Auto-Neg. Link Partner Abitily     */
#define PHY_REG_ANE         0x06        /* Auto-Neg. Expansion                */
#define PHY_REG_ANNP        0x07        /* Auto-Neg. Next Page TX             */
#define PHY_REG_LPNPA       0x08        /* Link Partner Next Page Ability     */
#define PHY_REG_RXERC       0x15        /* RXER Counter                       */
#define PHY_REG_ICS         0x1B        /* Interrupt Control/Status           */
#define PHY_REG_PC1         0x1E        /* PHY Control 1                      */
#define PHY_REG_PC2         0x1F        /* PHY Control 2                      */

/* Duplex and speed modes */
#define PHY_CON_10M         0x0001
#define PHY_CON_100M        0x0002
#define PHY_CON_HD          0x0004
#define PHY_CON_FD          0x0008

#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit                */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit                */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                 */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                 */
#define PHY_AUTO_NEG        0x1000      /* Select Auto Negotiation            */

#define PHY_DEF_ADDR        0x00        /* Default PHY device address         */
#define PHY_ID_DP83848C     0x20005C90  /* DP83848C PHY Identifier            */
#define PHY_ID_ST802RT1     0x02038460  /* ST802RT1x PHY Identifier           */
#define PHY_ID_KSZ8041      0x00221510  /* KSZ8041x PHY Identifier            */

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;

typedef struct os_frame {         /* << System frame buffer structure >>     */
  U16 length;                     /* Total Length of data in frame           */
  U16 index;                      /* Buffer Position Index                   */
  U8  data[1555];                    /* Buffer data (protocol headers + data)   */
} OS_FRAME;


#ifdef __cplusplus
}
#endif

#endif
