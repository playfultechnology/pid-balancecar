#ifndef _DMA_H_
#define _DMA_H_


typedef enum DMA_sources
{
    /*        ����ͨ��            */
    Channel_Disabled    = 0,

    /*        UART            */
    DMA_UART0_Rx            = 2,
    DMA_UART0_Tx            = 3,
    DMA_UART1_Rx            = 4,
    DMA_UART1_Tx            = 5,
    DMA_UART2_Rx            = 6,
    DMA_UART2_Tx            = 7,
    DMA_UART3_Rx            = 8,
    DMA_UART3_Tx            = 9,
    DMA_UART4_Rx            = 10,
    DMA_UART4_Tx            = 11,
    DMA_UART5_Rx            = 12,
    DMA_UART5_Tx            = 13,

    /*        I2S            */
    DMA_I2S0_Rx             = 14,
    DMA_I2S0_Tx             = 15,

    /*        SPI            */
    DMA_SPI0_Rx             = 16,
    DMA_SPI0_Tx             = 17,
    DMA_SPI1_Rx             = 18,
    DMA_SPI1_Tx             = 19,

    /*        I2C            */
    DMA_I2C0                = 22,
    DMA_I2C1                = 23,

    /*        FTM            */
    DMA_FTM0_CH0            = 24,
    DMA_FTM0_CH1            = 25,
    DMA_FTM0_CH2            = 26,
    DMA_FTM0_CH3            = 27,
    DMA_FTM0_CH4            = 28,
    DMA_FTM0_CH5            = 29,
    DMA_FTM0_CH6            = 30,
    DMA_FTM0_CH7            = 31,

    DMA_FTM1_CH0            = 32,
    DMA_FTM1_CH1            = 33,

    DMA_FTM2_CH0            = 34,
    DMA_FTM2_CH1            = 35,

    DMA_FTM3_CH0            = 36,
    DMA_FTM3_CH1            = 37,
    DMA_FTM3_CH2            = 38,

    DMA_FTM1_CH3            = 39,       //��ô���� FTM1 �أ�datasheet��������

    /*     ADC/DAC/CMP/CMT    */
    DMA_ADC0                = 40,
    DMA_ADC1                = 41,
    DMA_CMP0                = 42,
    DMA_CMP1                = 43,
    DMA_CMP2                = 44,
    DMA_DAC0                = 45,
    DMA_DAC1                = 46,
    DMA_CMT                 = 47,

    DMA_PDB                 = 48,


    DMA_Port_A              = 49,
    DMA_Port_B              = 50,
    DMA_Port_C              = 51,
    DMA_Port_D              = 52,
    DMA_Port_E              = 53,


    DMA_FTM3_CH4            = 54,
    DMA_FTM3_CH5            = 55,
    DMA_FTM3_CH6            = 56,
    DMA_FTM3_CH7            = 57,

    DMA_Always_EN1          = 58,
    DMA_Always_EN2          = 59,
    DMA_Always_EN3          = 60,
    DMA_Always_EN4          = 61,
    DMA_Always_EN5          = 62,
    DMA_Always_EN6          = 63,
} DMA_sources;



typedef enum DMA_PORTx2BUFF_cfg
{
    DMA_rising          = 0x01u,            //�����ش���
    DMA_falling         = 0x02u,            //�½��ش���
    DMA_either          = 0x03u,            //�����ش���

    //�������λ��־����������
    DMA_rising_down     = 0x81u,            //�����ش�����Դ��ַIO�˿��ڲ�����
    DMA_falling_down    = 0x82u,            //�½��ش�����Դ��ַIO�˿��ڲ�����
    DMA_either_down     = 0x83u,            //�����ش�����Դ��ַIO�˿��ڲ�����

    DMA_rising_up       = 0xc1u,            //�����ش�����Դ��ַIO�˿��ڲ�����
    DMA_falling_up      = 0xc2u,            //�½��ش�����Դ��ַIO�˿��ڲ�����
    DMA_either_up       = 0xc3u,            //�����ش�����Դ��ַIO�˿��ڲ�����

    //��λ6����־�����������,Ŀ�ĵ�ַ���ֲ��䣬���ָ���ԭ����ַ
    DMA_rising_keepon          = 0x21u,     //�����ش���                      ��Ŀ�ĵ�ַ���ֲ���
    DMA_falling_keepon         = 0x22u,     //�½��ش���                      ��Ŀ�ĵ�ַ���ֲ���
    DMA_either_keepon          = 0x23u,     //�����ش���                      ��Ŀ�ĵ�ַ���ֲ���

    DMA_rising_down_keepon     = 0xA1u,     //�����ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���
    DMA_falling_down_keepon    = 0xA2u,     //�½��ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���
    DMA_either_down_keepon     = 0xA3u,     //�����ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���

    DMA_rising_up_keepon       = 0xF1u,     //�����ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���
    DMA_falling_up_keepon      = 0xF2u,     //�½��ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���
    DMA_either_up_keepon       = 0xF3u,     //�����ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���

} DMA_PORTx2BUFF_cfg, DMA_Count_cfg;

typedef enum DMA_BYTEn      //DMAÿ�δ����ֽ���
{
    DMA_BYTE1 = 0,
    DMA_BYTE2 = 1,
    DMA_BYTE4 = 2,
    DMA_BYTE16 = 4
} DMA_BYTEn;


typedef enum DMA_CHn
{
    DMA_CH0,
    DMA_CH1,
    DMA_CH2,
    DMA_CH3,
    DMA_CH4,
    DMA_CH5,
    DMA_CH6,
    DMA_CH7,
    DMA_CH8,
    DMA_CH9,
    DMA_CH10,
    DMA_CH11,
    DMA_CH12,
    DMA_CH13,
    DMA_CH14,
    DMA_CH15
} DMA_CHn;

//����PTA��8λ����˿�
#define PTA_BYTE0_IN   PTA_BASE_PTR->PDIRByte.Byte0
#define PTA_BYTE1_IN   PTA_BASE_PTR->PDIRByte.Byte1
#define PTA_BYTE2_IN   PTA_BASE_PTR->PDIRByte.Byte2
#define PTA_BYTE3_IN   PTA_BASE_PTR->PDIRByte.Byte3

//����PTB��8λ����˿�
#define PTB_BYTE0_IN   PTB_BASE_PTR->PDIRByte.Byte0
#define PTB_BYTE1_IN   PTB_BASE_PTR->PDIRByte.Byte1
#define PTB_BYTE2_IN   PTB_BASE_PTR->PDIRByte.Byte2
#define PTB_BYTE3_IN   PTB_BASE_PTR->PDIRByte.Byte3

//����PTC��8λ����˿�
#define PTC_BYTE0_IN   PTC_BASE_PTR->PDIRByte.Byte0
#define PTC_BYTE1_IN   PTC_BASE_PTR->PDIRByte.Byte1
#define PTC_BYTE2_IN   PTC_BASE_PTR->PDIRByte.Byte2
#define PTC_BYTE3_IN   PTC_BASE_PTR->PDIRByte.Byte3


//����PTD��8λ����˿�
#define PTD_BYTE0_IN   PTD_BASE_PTR->PDIRByte.Byte0
#define PTD_BYTE1_IN   PTD_BASE_PTR->PDIRByte.Byte1
#define PTD_BYTE2_IN   PTD_BASE_PTR->PDIRByte.Byte2
#define PTD_BYTE3_IN   PTD_BASE_PTR->PDIRByte.Byte3

//����PTE��8λ����˿�
#define PTE_BYTE0_IN   PTE_BASE_PTR->PDIRByte.Byte0
#define PTE_BYTE1_IN   PTE_BASE_PTR->PDIRByte.Byte1
#define PTE_BYTE2_IN   PTE_BASE_PTR->PDIRByte.Byte2
#define PTE_BYTE3_IN   PTE_BASE_PTR->PDIRByte.Byte3


#define  DMA_IRQ_EN(DMA_CHn)    NVIC_EnableIRQ((DMA_CHn) + 0)                         //����DMAͨ������
#define  DMA_IRQ_DIS(DMA_CHn)   NVIC_DisableIRQ((DMA_CHn) + 0)                        //��ֹDMAͨ������
#define  DMA_IRQ_CLEAN(DMA_CHn) DMA_INT|=(DMA_INT_INT0_MASK<<DMA_CHn)                 //���ͨ�������жϱ�־λ

#define  DMA_EN(DMA_CHn)        DMA_ERQ |= (DMA_ERQ_ERQ0_MASK<<(DMA_CHn))             //ʹ��ͨ��Ӳ��DMA����
#define  DMA_DIS(DMA_CHn)       DMA_ERQ &=~(DMA_ERQ_ERQ0_MASK<<(DMA_CHn))             //��ֹͨ��Ӳ��DMA����

void DMA_PORTx2BUFF_Init(DMA_CHn CHn,
                         void *SADDR,
                         void *DADDR,
                         PTXn_e ptxn,
                         DMA_BYTEn byten,
                         uint32_t count,
                         DMA_PORTx2BUFF_cfg cfg);//DMA��ʼ������ȡ�˿����ݵ��ڴ�


void DMATransDataStart(uint8_t CHn,uint32_t address);//����DMA�����Ŀ�ĵ�ַ

void DMA_Count_Init(DMA_CHn CHn, PTXn_e ptxn, u32 count, DMA_PORTx2BUFF_cfg cfg);//����˵����DMA�ۼӼ�����ʼ��


u32  DMA_Count_Get(DMA_CHn CHn);//��ȡ�ۼӼ���ֵ


void DMA_Count_Reset(DMA_CHn CHn);//����ۼӼ���ֵ


#endif