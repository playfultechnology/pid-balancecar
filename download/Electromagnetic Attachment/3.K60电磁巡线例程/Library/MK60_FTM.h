
#ifndef _FTM_H_
#define _FTM_H_

/**********************************  FTM(引脚复用) ***************************************/
//   PWM输出通道    端口          可选范围              建议
#define FTM0_CH0    PTC1        //PTC1、PTA3            PTA3不要用（与JLINK冲突）
#define FTM0_CH1    PTC2        //PTC2、PTA4
#define FTM0_CH2    PTC3        //PTC3、PTA5
#define FTM0_CH3    PTC4        //PTC4、PTA6
#define FTM0_CH4    PTA7        //PTD4、PTA7
#define FTM0_CH5    PTD5        //PTD5、PTA0            PTA0不要用（与JLINK冲突）
#define FTM0_CH6    PTD6        //PTD6、PTA1            PTA1不要用（与JLINK冲突）
#define FTM0_CH7    PTD7        //PTD7、PTA2            PTA2不要用（与JLINK冲突）

//   PWM输出通道    端口          可选范围              建议
#define FTM1_CH0    PTA8       //PTA8、PTA12、PTB0
#define FTM1_CH1    PTA9       //PTA9、PTA13、PTB1

//   PWM输出通道    端口          可选范围              建议
#define FTM2_CH0    PTB18       //PTA10、PTB18
#define FTM2_CH1    PTB19       //PTA11、PTB19

//正交解码模块通道  端口          可选范围              建议
#define FTM1_QDPHA  PTA8       //PTA8、PTA12、PTB0
#define FTM1_QDPHB  PTA9       //PTA9、PTA13、PTB1

#define FTM2_QDPHA  PTB18       //PTA10、PTB18
#define FTM2_QDPHB  PTB19       //PTA11、PTB19

#ifdef MK60FX
//      模块通道    端口          可选范围
#define FTM3_CH0    PTE5        // PTE5、PTD0
#define FTM3_CH1    PTE6        // PTE6、PTD1
#define FTM3_CH2    PTE7        // PTE7、PTD2
#define FTM3_CH3    PTE8        // PTE8、PTD3
#define FTM3_CH4    PTE9        // PTE9、PTC8
#define FTM3_CH5    PTE10       // PTE10、PTC9
#define FTM3_CH6    PTE11       // PTE11、PTC10
#define FTM3_CH7    PTE12       // PTE12、PTC11
#endif

#define FTM_PRECISON  7200

//定义FTM模块号
typedef enum
{
    FTM0,
    FTM1,
    FTM2,
#ifdef MK60FX
    FTM3,
#endif
    FTM_MAX,
} FTMn_e;

//定义FTM 通道号
typedef enum
{
    FTM_CH0,
    FTM_CH1,
    FTM_CH2,
    FTM_CH3,
    FTM_CH4,
    FTM_CH5,
    FTM_CH6,
    FTM_CH7,

} FTM_CHn_e;

typedef enum
{
    FTM_Rising,               //上升沿捕捉
    FTM_Falling,              //下降沿捕捉
    FTM_Rising_or_Falling     //跳变沿捕捉
} FTM_Input_cfg;

#define FTM_IRQ_EN(FTMn,CHn)        FTM_CnSC_REG(FTMN[FTMn],CHn) |= FTM_CnSC_CHIE_MASK       //开启 FTMn_CHn 中断
#define FTM_IRQ_DIS(FTMn,CHn)       FTM_CnSC_REG(FTMN[FTMn],CHn) &= ~FTM_CnSC_CHIE_MASK      //关闭 FTMn_CHn 中断

void FTM_PwmInit(FTMn_e ftmn, FTM_CHn_e ch, uint16_t freq, uint16_t duty);

void FTM_PwmDuty(FTMn_e ftmn, FTM_CHn_e ch, u16 duty);

void FTM_ABInit(FTMn_e ftmn);

short FTM_ABGet(FTMn_e ftmn);

#endif