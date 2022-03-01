/**
  ******************************************************************************
  * @file    ftm.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����K60�̼��� FTM ��ʱ�� ���� �����ļ�
  ******************************************************************************
  */
#include "ftm.h"

/***********************************************************************************************
 ���ܣ�����PWM����ģʽ
 �βΣ�FTM_PWM_InitTypeDef:PWM��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
static void FTM_PWM_SetMode(FTM_InitTypeDef *FTM_InitStruct)
{
	FTM_Type *FTMx = NULL;
	FTM_PWM_MapTypeDef *pFTM_Map = NULL;
	pFTM_Map = (FTM_PWM_MapTypeDef*)&(FTM_InitStruct->FTMxMAP);
	//�ҳ�FTM��
	switch(pFTM_Map->FTM_Index)
	{
		case 0:
			FTMx = FTM0;
			break;
		case 1:
			FTMx = FTM1;
			break;
		case 2:
			FTMx = FTM2;
			break;
		default:break;	
	}
	switch(FTM_InitStruct->FTM_Mode)
	{
		case FTM_Mode_EdgeAligned: //���ض���ģʽ
			//��ֹд����
			FTMx->MODE |= FTM_MODE_WPDIS_MASK;
		  //��ֹ��ǿģʽ
			FTMx->MODE &= ~FTM_MODE_FTMEN_MASK;
		  //��ֹ��������ģʽ
			FTMx->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
		  //���ض���
			FTMx->SC &= ~FTM_SC_CPWMS_MASK;
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC = 0;
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK);	
			switch(pFTM_Map->FTM_CH_Index)
			{
				case 0:
				case 1:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE0_MASK;	
					break;
				case 2:
				case 3:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN1_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE1_MASK;					
					break;
				case 4:
				case 5:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN2_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE2_MASK;	
					break;
				case 6:
				case 7:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN3_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE3_MASK;	
					break;
				default:break;
			}
			break;
		case FTM_Mode_CenterAligned: //���ж���ģʽ
			//��ֹд���� ����д������FTM�Ĵ��� ��ֹ��ǿģʽ
			FTMx->MODE |= FTM_MODE_WPDIS_MASK;
			FTMx->MODE &= ~FTM_MODE_FTMEN_MASK;
			FTMx->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
			FTMx->SC |= FTM_SC_CPWMS_MASK;
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC = 0;
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK);	
			switch(pFTM_Map->FTM_CH_Index)
			{
				case 0:
				case 1:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE0_MASK;	
					break;
				case 2:
				case 3:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN1_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE1_MASK;					
					break;
				case 4:
				case 5:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN2_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE2_MASK;	
					break;
				case 6:
				case 7:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN3_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_COMBINE3_MASK;	
					break;
				default:break;
			}
			break;
		case FTM_Mode_Combine:  //���ģʽ Chl(n) & Chl(n+1) ��� ��Chl(n) ���
			//��ֹд���� ����д������FTM�Ĵ��� ������ǿģʽ
			FTMx->MODE |= FTM_MODE_WPDIS_MASK;
			FTMx->MODE |= FTM_MODE_FTMEN_MASK;
			FTMx->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
			FTMx->SC &= ~FTM_SC_CPWMS_MASK;
		  //��������
			//FTMx->DEADTIME=FTM_DEADTIME_DTPS(2)|FTM_DEADTIME_DTVAL(10);
		  //ͬ������
			FTMx->SYNC = FTM_SYNC_CNTMIN_MASK|FTM_SYNC_CNTMAX_MASK;
			FTMx->SYNC |= FTM_SYNC_SWSYNC_MASK;
		  //װ�����ֵ
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC = 0;
			FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK);	
			switch(pFTM_Map->FTM_CH_Index)
			{
				case 0:
				case 1:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;
					FTMx->COMBINE |= FTM_COMBINE_COMBINE0_MASK;	
					FTMx->COMBINE &= ~FTM_COMBINE_COMP0_MASK;
				  FTMx->COMBINE |= FTM_COMBINE_SYNCEN0_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_DTEN0_MASK;
					break;
				case 2:
				case 3:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN1_MASK;
					FTMx->COMBINE |= FTM_COMBINE_COMBINE1_MASK;	
					FTMx->COMBINE &= ~FTM_COMBINE_COMP1_MASK;
				  FTMx->COMBINE |= FTM_COMBINE_SYNCEN1_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_DTEN1_MASK;
					break;
				case 4:
				case 5:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN2_MASK;
					FTMx->COMBINE |= FTM_COMBINE_COMBINE2_MASK;	
					FTMx->COMBINE &= ~FTM_COMBINE_COMP2_MASK;
				  FTMx->COMBINE |= FTM_COMBINE_SYNCEN2_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_DTEN2_MASK;
					break;
				case 6:
				case 7:
					FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN3_MASK;
					FTMx->COMBINE |= FTM_COMBINE_COMBINE3_MASK;	
					FTMx->COMBINE &= ~FTM_COMBINE_COMP3_MASK;
				  FTMx->COMBINE |= FTM_COMBINE_SYNCEN3_MASK;
					FTMx->COMBINE &= ~FTM_COMBINE_DTEN3_MASK;
					break;
				default:break;
			}
			break;
			case FTM_Mode_Complementary:
			{
				//��ֹд���� ����д������FTM�Ĵ��� ������ǿģʽ
				FTMx->MODE |= FTM_MODE_WPDIS_MASK;
				FTMx->MODE |= FTM_MODE_FTMEN_MASK;
				FTMx->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
				FTMx->SC &= ~FTM_SC_CPWMS_MASK;
				//��������
				FTMx->DEADTIME=FTM_DEADTIME_DTPS(2)|FTM_DEADTIME_DTVAL(5);
				//ͬ������
				FTMx->SYNC = FTM_SYNC_CNTMIN_MASK|FTM_SYNC_CNTMAX_MASK;
				FTMx->SYNC |= FTM_SYNC_SWSYNC_MASK;
				//װ�����ֵ
				FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC = 0;
				FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnSC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK);
				switch(pFTM_Map->FTM_CH_Index)
				{
					case 0:
					case 1:
						FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN0_MASK;
						FTMx->COMBINE |= FTM_COMBINE_COMBINE0_MASK;	
						FTMx->COMBINE |= FTM_COMBINE_COMP0_MASK;
						FTMx->COMBINE |= FTM_COMBINE_SYNCEN0_MASK;
						FTMx->COMBINE |= FTM_COMBINE_DTEN0_MASK;
						break;
					case 2:
					case 3:
						FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN1_MASK;
						FTMx->COMBINE |= FTM_COMBINE_COMBINE1_MASK;	
						FTMx->COMBINE |= FTM_COMBINE_COMP1_MASK;
						FTMx->COMBINE |= FTM_COMBINE_SYNCEN1_MASK;
						FTMx->COMBINE |= FTM_COMBINE_DTEN1_MASK;
						break;
					case 4:
					case 5:
						FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN2_MASK;
						FTMx->COMBINE |= FTM_COMBINE_COMBINE2_MASK;	
						FTMx->COMBINE |= FTM_COMBINE_COMP2_MASK;
						FTMx->COMBINE |= FTM_COMBINE_SYNCEN2_MASK;
						FTMx->COMBINE |= FTM_COMBINE_DTEN2_MASK;
						break;
					case 6:
					case 7:
						FTMx->COMBINE &= ~FTM_COMBINE_DECAPEN3_MASK;
						FTMx->COMBINE |= FTM_COMBINE_COMBINE3_MASK;	
						FTMx->COMBINE |= FTM_COMBINE_COMP3_MASK;
						FTMx->COMBINE |= FTM_COMBINE_SYNCEN3_MASK;
						FTMx->COMBINE |= FTM_COMBINE_DTEN3_MASK;
						break;
					default:break;
				}
			}
			break;
			default:break;
	}
}
/***********************************************************************************************
 ���ܣ��ı�PWMͨ��ռ�ձ�
 �βΣ�FTMxMAP:PWM ͨ��ѡ��
			 @arg  FTM0_CH0_PC1: FTM0ģ�� 0 ͨ�� PC1����
			 @arg  FTM0_CH0_PA3: FTM0ģ�� 0 ͨ�� PA3����
			 @arg  ...
			 PWMDuty: ռ�ձ�0-10000 ��Ӧ0-100%
 ���أ�0
 ��⣺0
************************************************************************************************/
void FTM_PWM_ChangeDuty(uint32_t FTMxMAP,uint32_t PWMDuty)
{
	uint32_t cv = 0;
	FTM_Type *FTMx = NULL;
	FTM_PWM_MapTypeDef* pFTM_Map = (FTM_PWM_MapTypeDef*) &FTMxMAP;
	//������
	assert_param(IS_FTM_PWM_MAP(FTMxMAP));
	assert_param(IS_FTM_PWM_DUTY(PWMDuty));
	
	switch(pFTM_Map->FTM_Index)
	{
		case 0:
			FTMx = FTM0;
			break;
		case 1:
			FTMx = FTM1;
			break;
		case 2:
			FTMx = FTM2;
			break;
		default:break;
	}
	cv = ((FTMx->MOD)*(PWMDuty))/10000; //����Ƚ�ֵ
	FTMx->CONTROLS[pFTM_Map->FTM_CH_Index].CnV = cv; 
}
/***********************************************************************************************
 ���ܣ���ʼ��PWM����
 �βΣ�FTM_PWM_InitTypeDef PWM��ʼ���ṹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void FTM_Init(FTM_InitTypeDef *FTM_InitStruct)
{
	uint8_t prescaler;   //��Ƶ����
	uint32_t mod;
	FTM_Type *FTMx = NULL;
	PORT_Type *FTM_PORT = NULL;
	FTM_PWM_MapTypeDef *pFTM_Map = (FTM_PWM_MapTypeDef*)&(FTM_InitStruct->FTMxMAP);
	
	//������
	assert_param(IS_FTM_PWM_MAP(FTM_InitStruct->FTMxMAP));
	assert_param(IS_FTM_PWM_MODE(FTM_InitStruct->FTM_Mode));
	assert_param(IS_FTM_PWM_DUTY(FTM_InitStruct->InitalDuty));
	
	//�ҳ�FTM��
	switch(pFTM_Map->FTM_Index)
	{
		case 0:
			SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
			FTMx = FTM0;
			break;
		case 1:
			SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
			FTMx = FTM1;
			break;
		case 2:
			SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
			FTMx = FTM2;
			break;
		default:break;	
	}
	//�ҳ�PORT�˿�
	switch(pFTM_Map->FTM_GPIO_Index)
	{
		case 0:
			FTM_PORT = PORTA;
			SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
			break;
		case 1:
			FTM_PORT = PORTB;
			SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
			break;	
		case 2:
			FTM_PORT = PORTC;
			SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
			break;
		case 3:
			FTM_PORT = PORTD;
			SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
			break;
		case 4:
			FTM_PORT = PORTE;
			SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
			break;
	}
	//���ö�Ӧ��IO��ΪPWMģʽ
	FTM_PORT->PCR[pFTM_Map->FTM_Pin_Index] &= ~PORT_PCR_MUX_MASK;
	FTM_PORT->PCR[pFTM_Map->FTM_Pin_Index] |= PORT_PCR_MUX(pFTM_Map->FTM_Alt_Index);
  //�����Ƶ��MODE���� 
	prescaler = (CPUInfo.BusClock/(FTM_InitStruct->Frequency))/65535;
 //PS>4ʱ�ܻ����
	if(prescaler >= 4 ) prescaler = 4;
	//����MODEװ�ز���
	mod = (CPUInfo.BusClock/((FTM_InitStruct->Frequency)*(1<<prescaler)));
  //ʱ��Դ����Ƶѡ��
	FTMx->SC &=~ FTM_SC_CLKS_MASK;
	FTMx->SC &= ~FTM_SC_PS_MASK;
	FTMx->SC |= (FTM_SC_CLKS(1)| FTM_SC_PS(prescaler));         
	
  //����PWM���ڼ�ռ�ձ�
	FTMx->MOD = mod;                     //���������ֵ
	FTMx->CNTIN = 0x0000u;	             //��������ʼ��ʱ�ļ���ֵ
	FTMx->CNT = 0x0000u;                 //��������ʼ��ֵ
	//�ʵ���ʱ
	for(mod=0;mod < 400000;mod++){}; //���ʵ���ʱ �ȴ�Ӳ��ģ����������
	//���ø���ͨ����ģʽ
	 FTM_PWM_SetMode(FTM_InitStruct);
  //���ó�ʼռ�ձ�	
	FTM_PWM_ChangeDuty(FTM_InitStruct->FTMxMAP,FTM_InitStruct->InitalDuty);
}

/***********************************************************************************************
 ���ܣ�FTM ���������������
 �βΣ�ftm: FTMģ���
       value: �������ֵָ��
       dir��  ����ָ�� 0���� 1���� 
 ���أ�0
 ��⣺Added in 2013-12-12
************************************************************************************************/
void FTM_QDGetData(FTM_Type *ftm, uint32_t* value, uint8_t* dir)
{
	*dir = (((ftm->QDCTRL)>>FTM_QDCTRL_QUADIR_SHIFT)&1);
	*value = (ftm->CNT);
}

/***********************************************************************************************
 ���ܣ�FTM ���������� ��ʼ�� 
 �βΣ�FTM1_QD_A12_PHA_A13_PHB: FTM1 A12-PHA  A13-PHB
       FTM1_QD_B00_PHA_B01_PHB
       FTM2_QD_B18_PHA_B19_PHB
 ���أ�0
 ��⣺Added in 2013-12-12
************************************************************************************************/
void FTM_QDInit(uint32_t FTM_QD_Maps)
{
    FTM_Type *FTMx = NULL;
    PORT_Type *FTM_PORT = NULL;
    FTM_QD_MapTypeDef *pFTM_Map = (FTM_QD_MapTypeDef*)&FTM_QD_Maps;
    // get module index
    switch(pFTM_Map->FTM_Index)
    {
        case 0:
            SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
            FTMx = FTM0;
            break;
        case 1:
            SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
            FTMx = FTM1;
            break;
        case 2:
            SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
            FTMx = FTM2;
            break;
        default:
            break;	
    }
    //get pinmux port index
    switch(pFTM_Map->FTM_GPIO_Index)
    {
        case 0:
            FTM_PORT = PORTA;
            SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;
            break;
        case 1:
            FTM_PORT = PORTB;
            SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;
            break;	
        case 2:
            FTM_PORT = PORTC;
            SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;
            break;
        case 3:
            FTM_PORT = PORTD;
            SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;
            break;
        case 4:
            FTM_PORT = PORTE;
            SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
            break;
    }
    //Config the PinMux and enable pull up
    FTM_PORT->PCR[pFTM_Map->FTM_PHA_Index] &= ~PORT_PCR_MUX_MASK;
    FTM_PORT->PCR[pFTM_Map->FTM_PHA_Index] |= PORT_PCR_MUX(pFTM_Map->FTM_Alt_Index);
    FTM_PORT->PCR[pFTM_Map->FTM_PHA_Index] |= PORT_PCR_PE_MASK;
    FTM_PORT->PCR[pFTM_Map->FTM_PHA_Index] |= PORT_PCR_PS_MASK;
		
    FTM_PORT->PCR[pFTM_Map->FTM_PHB_Index] &= ~PORT_PCR_MUX_MASK;
    FTM_PORT->PCR[pFTM_Map->FTM_PHB_Index] |= PORT_PCR_MUX(pFTM_Map->FTM_Alt_Index);
    FTM_PORT->PCR[pFTM_Map->FTM_PHB_Index] |= PORT_PCR_PE_MASK;
    FTM_PORT->PCR[pFTM_Map->FTM_PHB_Index] |= PORT_PCR_PS_MASK;
		
    FTMx->MOD = 14000; //������Ҫ����
    FTMx->CNTIN = 0;
    FTMx->MODE |= FTM_MODE_WPDIS_MASK; //��ֹд����
    FTMx->MODE |= FTM_MODE_FTMEN_MASK; //FTMEN=1,�ر�TPM����ģʽ������FTM���й���
    FTMx->QDCTRL &= ~FTM_QDCTRL_QUADMODE_MASK; //ѡ������ģʽΪA����B�����ģʽ 
    FTMx->QDCTRL |= FTM_QDCTRL_QUADEN_MASK; //ʹ����������ģʽ
    //����ʱ��
    FTMx->SC |= FTM_SC_CLKS(1)|FTM_SC_PS(3);
}


/*

static const FTM_QD_MapTypeDef FTM_PWM_Check_Maps[] = 
{
    {1, 0, 12 ,13, 0, 7}, //FTM1_QD_A12_PHA_A13_PHB
    {1, 0,  0 , 1, 1, 6}, //FTM1_QD_B00_PHA_B01_PHB
    {2, 0, 18 ,19, 1, 6}, //FTM2_QD_B18_PHA_B19_PHB
};
void PWM_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(FTM_PWM_Check_Maps)/sizeof(FTM_QD_MapTypeDef);i++)
	{
		value = FTM_PWM_Check_Maps[i].FTM_Index<<0;
		value|= FTM_PWM_Check_Maps[i].FTM_CH_Index<<4;
		value|= FTM_PWM_Check_Maps[i].FTM_PHA_Index<<8;
		value|= FTM_PWM_Check_Maps[i].FTM_PHB_Index<<14;
		value|= FTM_PWM_Check_Maps[i].FTM_GPIO_Index<<20;
		value|= FTM_PWM_Check_Maps[i].FTM_Alt_Index<<24;
		UART_printf("(0x%xU)\r\n",value);
	}
}
//����
static const FTM_PWM_MapTypeDef FTM_PWM_Check_Maps[] = 
{ 
    {0, 0, 1, 2, 4},  //FTM0_CH0_PC1
    {0, 0, 3, 0, 3},  //FTM0_CH0_PA3
    {0, 1, 2, 2, 4},  //FTM0_CH1_PC2
    {0, 1, 4, 0, 3},  //FTM0_CH1_PA4
    {0, 2, 3, 2, 4},  //FTM0_CH2_PC3
    {0, 2, 5, 0, 3},  //FTM0_CH2_PA5
    {0, 3, 4, 2, 4},  //FTM0_CH3_PC4
    {0, 4, 4, 3, 4},  //FTM0_CH4_PD4
    {0, 5, 5, 3, 4},  //FTM0_CH5_PD5
    {0, 5, 0, 0, 3},  //FTM0_CH5_PA0
    {0, 6, 6, 3, 4},  //FTM0_CH6_PD6
    {0, 6, 1, 0, 3},  //FTM0_CH6_PA1
    {0, 7, 7, 3, 4},  //FTM0_CH7_PD7
    {0, 7, 2, 0, 3},  //FTM0_CH7_PA2
    {1, 0,12, 0, 3},  //FTM1_CH0_PA12
    {1, 0, 0, 1, 3},  //FTM1_CH0_PB0
    {1, 1,13, 0, 3},  //FTM1_CH1_PA13
    {1, 1, 1, 1, 3},  //FTM1_CH1_PB1
    {2, 0,18, 1, 3},  //FTM2_CH0_PB18
    {2, 1,19, 1, 3},  //FTM2_CH1_PB19
};

void PWM_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(FTM_PWM_Check_Maps)/sizeof(FTM_PWM_MapTypeDef);i++)
	{
		value = FTM_PWM_Check_Maps[i].FTM_Index<<0;
		value|= FTM_PWM_Check_Maps[i].FTM_CH_Index<<4;
		value|= FTM_PWM_Check_Maps[i].FTM_Pin_Index<<8;
		value|= FTM_PWM_Check_Maps[i].FTM_GPIO_Index<<14;
		value|= FTM_PWM_Check_Maps[i].FTM_Alt_Index<<18;
		printf("(0x%08xU)\r\n",value);
	}
}
*/
/***********************************************************************************************
 ���ܣ�FTM_ITConfig
 �βΣ�FTMx : FTMģ��ͨ�� 
       @arg FTM0 : FTM0ģ��
			 @arg FTM1 : FTM1ģ��
			 @arg FTM2 : FTM2ģ��
			 FTM_IT �� FTM�ж�Դ
			 @arg FTM_IT_TOF          
			 @arg FTM_IT_CHF0          
			 @arg FTM_IT_CHF1          
			 @arg FTM_IT_CHF2          
			 @arg FTM_IT_CHF3          
			 @arg FTM_IT_CHF4          
			 @arg FTM_IT_CHF5          
			 @arg FTM_IT_CHF6          
			 @arg FTM_IT_CHF7         
			 NewState: �رջ���ʹ��
			 @arg ENABLE : ʹ��
			 @arg DISABLE:��ֹ
 ���أ�0
 ��⣺0
************************************************************************************************/
void FTM_ITConfig(FTM_Type* FTMx, uint16_t FTM_IT, FunctionalState NewState)
{
	//�������
	assert_param(IS_FTM_ALL_PERIPH(FTMx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	switch(FTM_IT)
	{
		case FTM_IT_TOF:
			(ENABLE == NewState)?(FTMx->SC |= FTM_SC_TOIE_MASK):(FTMx->SC &= ~FTM_SC_TOIE_MASK);
			break;
		case FTM_IT_CHF0:
		case FTM_IT_CHF1:
		case FTM_IT_CHF2:
		case FTM_IT_CHF3:
		case FTM_IT_CHF4:
		case FTM_IT_CHF5:
		case FTM_IT_CHF6:
		case FTM_IT_CHF7:	
			(ENABLE == NewState)?(FTMx->CONTROLS[FTM_IT].CnSC |= FTM_CnSC_CHIE_MASK):(FTMx->CONTROLS[FTM_IT].CnSC &= ~FTM_CnSC_CHIE_MASK);		
			break;
		default:break;
	}
}
/***********************************************************************************************
 ���ܣ�FTM_GetITStatus ���IT��־λ
 �βΣ�FTMx : FTMģ��ͨ�� 
       @arg FTM0 : FTM0ģ��
			 @arg FTM1 : FTM1ģ��
			 @arg FTM2 : FTM2ģ��
			 FTM_IT �� FTM�ж�Դ
			 @arg FTM_IT_TOF          
			 @arg FTM_IT_CHF0          
			 @arg FTM_IT_CHF1          
			 @arg FTM_IT_CHF2          
			 @arg FTM_IT_CHF3          
			 @arg FTM_IT_CHF4          
			 @arg FTM_IT_CHF5          
			 @arg FTM_IT_CHF6          
			 @arg FTM_IT_CHF7         
 ���أ�ITStatus:��־
       @arg SET:��־λ��λ
       @arg RESET:��־λ��0
 ��⣺0
************************************************************************************************/
ITStatus FTM_GetITStatus(FTM_Type* FTMx, uint16_t FTM_IT)
{
	ITStatus retval;
	//�������
	assert_param(IS_FTM_ALL_PERIPH(FTMx));
	assert_param(IS_FTM_IT(FTM_IT));
	

	switch(FTM_IT)
	{
		case FTM_IT_TOF:
			(FTMx->SC & FTM_SC_TOF_MASK)?(retval = SET):(retval = RESET);
		break;
		case FTM_IT_CHF0:
		case FTM_IT_CHF1:
		case FTM_IT_CHF2:
		case FTM_IT_CHF3:
		case FTM_IT_CHF4:
		case FTM_IT_CHF5:
		case FTM_IT_CHF6:
		case FTM_IT_CHF7:	
			(FTMx->CONTROLS[FTM_IT].CnSC& FTM_CnSC_CHF_MASK)?(retval = SET):(retval = RESET);
			break;
	}
	return retval;
}
/***********************************************************************************************
 ���ܣ���IT ��־λ
 �βΣ�FTMx : FTMģ��ͨ�� 
       @arg FTM0 : FTM0ģ��
			 @arg FTM1 : FTM1ģ��
			 @arg FTM2 : FTM2ģ��
			 FTM_IT �� FTM�ж�Դ
			 @arg FTM_IT_TOF          
			 @arg FTM_IT_CHF0          
			 @arg FTM_IT_CHF1          
			 @arg FTM_IT_CHF2          
			 @arg FTM_IT_CHF3          
			 @arg FTM_IT_CHF4          
			 @arg FTM_IT_CHF5          
			 @arg FTM_IT_CHF6          
			 @arg FTM_IT_CHF7         
 ���أ�0
 ��⣺0
************************************************************************************************/
void FTM_ClearITPendingBit(FTM_Type *FTMx,uint16_t FTM_IT)
{
	uint32_t read_value = 0;
	//�������
	assert_param(IS_FTM_ALL_PERIPH(FTMx));
	assert_param(IS_FTM_IT(FTM_IT));
	
	read_value = read_value;
	switch(FTM_IT)
	{
		case FTM_IT_TOF:
			read_value = FTMx->SC;
		break;
		case FTM_IT_CHF0:
		case FTM_IT_CHF1:
		case FTM_IT_CHF2:
		case FTM_IT_CHF3:
		case FTM_IT_CHF4:
		case FTM_IT_CHF5:
		case FTM_IT_CHF6:
		case FTM_IT_CHF7:	
			read_value = FTMx->CONTROLS[FTM_IT].CnSC;
			break;
	}
}





