

#ifndef __GPIO_H__
#define __GPIO_H__

//����ܽŷ���
typedef enum GPIO_CFG
{
    //�����ֵ���ܸģ�����
    GPI         = 0,                          //����ܽ����뷽��      GPIOx_PDDRn�0��ʾ���룬1��ʾ���
    GPO         = 1,                          //����ܽ��������

    GPI_DOWN    = 0x02,                       //��������              PORTx_PCRn��ҪPE=1��PS=0
    GPI_UP      = 0x03,                       //��������              PORTx_PCRn��ҪPE=1��PS=1
    GPI_PF      = 0x10,                       //���룬����Դ�˲���,�˲���Χ��10 MHz ~ 30 MHz ����֧�ָ��ٽӿڣ�>=2MHz��  0b10000           Passive Filter Enable
    GPI_DOWN_PF = GPI_DOWN | GPI_PF ,         //��������������Դ�˲���
    GPI_UP_PF   = GPI_UP   | GPI_PF ,         //��������������Դ�˲���

    GPO_HDS     = 0x41,                        //�������������   0b100 0001    High drive strength
    GPO_SSR     = 0x05,                        //������仯��          0b101     Slow slew rate
    GPO_HDS_SSR = GPO_HDS | GPO_SSR,           //������������������仯��
} GPIO_CFG;  //���λΪ0���϶������룻GPI_UP �� GPI_UP_PF�����λΪ1������Ϊ���


typedef enum exti_cfg
{
    zero_down     = 0x08u,     //�͵�ƽ�������ڲ�����
    rising_down   = 0x09u,     //�����ش������ڲ�����
    falling_down  = 0x0Au,     //�½��ش������ڲ�����
    either_down   = 0x0Bu,     //�����ش������ڲ�����
    one_down      = 0x0Cu,     //�ߵ�ƽ�������ڲ�����

    //�����λ��־����������
    zero_up       = 0x88u,     //�͵�ƽ�������ڲ�����
    rising_up     = 0x89u,     //�����ش������ڲ�����
    falling_up    = 0x8Au,     //�½��ش������ڲ�����
    either_up     = 0x8Bu,     //�����ش������ڲ�����
    one_up        = 0x8Cu      //�ߵ�ƽ�������ڲ�����
} exti_cfg;

extern GPIO_MemMapPtr GPIOX[5];
extern PORT_MemMapPtr PORTX[5];

void GPIO_PinInit(PTXn_e ptx_n, GPIO_CFG dir, uint8_t data);//��ʼ��GPIO ������GPIOģʽ

void GPIO_PinWrite(PTXn_e ptx_n, uint8_t data);//дGPIO���

void GPIO_PinSetDir(PTXn_e ptx_n, uint8_t dir);//����IO����/���ģʽ

void GPIO_PinReverse(PTXn_e ptx_n);//GPIOȡ��

uint8_t GPIO_PinRead(PTXn_e ptx_n);//��ȡIO�ڵ�ƽ

void GPIO_ExtiInit(PTXn_e ptx_n, exti_cfg cfg);//GPIO�ⲿ�ж�
#endif
