#include "include.h"
#include "MK60_FTM.h"

#ifdef MK60FX
/* 定义四个指针数组保存 FTMn_e 的地址 */
FTM_MemMapPtr FTMN[4] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR, FTM3_BASE_PTR};
/* FTM输出PWM时的周期计数值 */
static uint32 ftm_period[4];
#else
/* 定义三个指针数组保存 FTMn_e 的地址 */
FTM_MemMapPtr FTMN[3] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR};
/* FTM输出PWM时的周期计数值 */
static uint32 ftm_period[3];
#endif


/* FTM引脚初始化 */
static void FTM_PinInit(FTMn_e ftmn, FTM_CHn_e ch, uint8_t mode);


/*------------------------------------------------------------------------------------------------------
【函    数】FTM_PwmInit
【功    能】初始化FTM模式PWM通道
【参    数】ftmn  :  模块名FTM0,FTM1，FTM2
【参    数】ch    :  通道号 对应引脚参考ftm.h文件
【参    数】freq  ： 设置PWM的频率
【参    数】duty  ： 设置PWM的占空比
【返 回 值】无
【实    例】FTM_PwmInit(FTM0, FTM_CH0, 10000, 500); //频率10KHZ，占空比为百分之（500/FTM_PRECISON *100）;
--------------------------------------------------------------------------------------------------------*/
void FTM_PwmInit(FTMn_e ftmn, FTM_CHn_e ch, uint16_t freq, uint16_t duty)
{

    /* 开启FTM时钟配置端口复用 */
    FTM_PinInit(ftmn, ch, 1);

    /* 配置FTM模块的最佳时钟 */
    uint32_t clk_hz = (bus_clk * 1000 * 1000) ;
    uint16_t mod = (clk_hz >> 16 ) / freq ;
    uint8_t ps = 0;
    while((mod >> ps) >= 1)             // 等 (mod >> ps) < 1 才退出 while 循环 ，即求 PS 的最小值
    {
        ps++;
    }

    if(ps>0x07) return ;               // 断言， PS 最大为 0x07 ，超过此值，则 PWM频率设置过低，或 Bus 频率过高
    mod = (clk_hz >> ps) / freq;        // 求 MOD 的值
    ftm_period[ftmn]=mod;
    uint16_t cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    FTM_SC_REG(FTMN[ftmn])   =  FTM_SC_PS(ps) | FTM_SC_CLKS(1);

    /******************** 选择输出模式为 边沿对齐PWM *******************/
    //通道状态控制，根据模式来选择 边沿或电平
    FTM_CnSC_REG(FTMN[ftmn], ch) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMN[ftmn], ch)  = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
   // FTM_MODE_REG(FTMN[ftmn])&=~1;
    FTM_MOD_REG(FTMN[ftmn])   = mod;                        //模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMN[ftmn]) = 0;                          //计数器初始化值。设置脉冲宽度：(CnV - CNTIN).
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;
    FTM_CNT_REG(FTMN[ftmn])   = 0;                          //计数器。只有低16位可用（写任何值到此寄存器，都会加载 CNTIN 的值）
}

/*------------------------------------------------------------------------------------------------------
【函    数】FTM_PwmDuty
【功    能】输出PWM波
【参    数】ftmn  :  模块名FTM0,FTM1，FTM2
【参    数】ch    :  通道号 对应引脚参考ftm.h文件
【参    数】duty  ： 设置PWM的占空比
【返 回 值】无
【实    例】FTM_PwmDuty(FTM0, FTM_CH0, 500); //占空比为百分之（500/FTM_PRECISON *100）;
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void FTM_PwmDuty(FTMn_e ftmn, FTM_CHn_e ch, u16 duty)
{
    uint32_t mod = ftm_period[ftmn];
    uint16_t cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    // 配置FTM通道值
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;

}


//////////////////////////////// 以下为正交解码 //////////////////////////////////////////



/*------------------------------------------------------------------------------------------------------
【函    数】FTM_ABInit
【功    能】初始化FTM模式正交解码模式
【参    数】ftmn  :  模块名FTM1，FTM2
【返 回 值】无
【实    例】FTM_ABInit(FTM1); //FTM1为正交解码模式
【注意事项】使用管脚在ftm.h文件查看
--------------------------------------------------------------------------------------------------------*/
void FTM_ABInit(FTMn_e ftmn)
{
   /* 开启FTM时钟配置端口复用 */
    FTM_PinInit(ftmn, FTM_CH0, 0);

    FTM_MODE_REG(FTMN[ftmn])  |=    (0
                                     | FTM_MODE_WPDIS_MASK  //写保护禁止
                                     //| FTM_MODE_FTMEN_MASK   //使能 FTM
                                    );
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADMODE_MASK
                                     );
    FTM_CNTIN_REG(FTMN[ftmn])   = 0;
    FTM_MOD_REG(FTMN[ftmn])     = FTM_MOD_MOD_MASK;
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADEN_MASK
                                     );
    FTM_MODE_REG(FTMN[ftmn])  |= FTM_QDCTRL_QUADEN_MASK;;
    FTM_CNT_REG(FTMN[ftmn])     = 0;                    //计数器。只有低16位可用（写任何值到此寄存器，都会加载 CNTIN 的值）

}


/*------------------------------------------------------------------------------------------------------
【函    数】FTM_ABGet
【功    能】获取正交解码的计数数值
【参    数】ftmn  :  模块名FTM1，FTM2
【返 回 值】计数值
【实    例】FTM_ABGet(FTM1); //FTM1正交解码值
【注意事项】使用管脚在ftm.h文件查看
--------------------------------------------------------------------------------------------------------*/
short FTM_ABGet(FTMn_e ftmn)
{
    short val;

    val = FTM_CNT_REG(FTMN[ftmn]);

    FTM_CNT_REG(FTMN[ftmn]) = 0;

    return val;
}



/*------------------------------------------------------------------------------------------------------
【函    数】FTM_PinInit
【功    能】初始化FTM管脚通道  内部调用
【参    数】ftmn  :  模块名FTM0,FTM1，FTM2
【参    数】ch    :  通道号 对应引脚参考ftm.h文件
【参    数】mode  ： 1：PWM管脚   0：正交解码管脚
【返 回 值】无
【实    例】FTM_PwmInit(FTM0, FTM_CH0, 1); //初始化FTM0的FTM_CH0通道为PWM模式
【注意事项】
--------------------------------------------------------------------------------------------------------*/
static void FTM_PinInit(FTMn_e ftmn, FTM_CHn_e ch, uint8_t mode)
{
    if(mode)
    {
        switch(ftmn)
        {
          case FTM0:
            SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;       //使能FTM0时钟

            switch(ch)
            {
              case FTM_CH0:
                if(FTM0_CH0 == PTC1)
                {
                    PORTC_PCR1= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH0 == PTA3)
                {
                    PORTA_PCR3= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH1:
                if(FTM0_CH1 == PTC2)
                {
                    // port_init(FTM0_CH1, ALT4);
                    PORTC_PCR2= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH1 == PTA4)
                {
                    PORTA_PCR4= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH2:
                if(FTM0_CH2 == PTC3)
                {
                    PORTC_PCR3= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH2 == PTA5)
                {
                    PORTA_PCR5= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH3:
                if(FTM0_CH3 == PTC4)
                {
                    PORTC_PCR4= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH3 == PTA6)
                {
                    PORTA_PCR6= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH4:
                if(FTM0_CH4 == PTD4)
                {
                    PORTD_PCR4= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH4 == PTA7)
                {
                    PORTA_PCR7= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH5:
                if(FTM0_CH5 == PTD5)
                {
                    PORTD_PCR5= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH5 == PTA0)
                {
                    PORTA_PCR0= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH6:
                if(FTM0_CH6 == PTD6)
                {
                    PORTD_PCR6= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH6 == PTA1)
                {
                    PORTA_PCR1= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;

              case FTM_CH7:
                if(FTM0_CH7 == PTD7)
                {
                    PORTD_PCR7= PORT_PCR_MUX(4);
                }
                else if(FTM0_CH7 == PTA2)
                {
                    PORTA_PCR2= PORT_PCR_MUX(3);
                }
                else
                {
                    break;
                }
                break;
              default:
                return;
            }
            break;

          case FTM1:
            SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //使能FTM1时钟
            switch(ch)
            {
              case FTM_CH0:
                if(FTM1_CH0 == PTA8)
                {
                    PORTA_PCR8= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH0 == PTA12)
                {
                    PORTA_PCR12= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH0 == PTB0)
                {
                    PORTB_PCR0= PORT_PCR_MUX(3);
                }
                break;


              case FTM_CH1:
                if(FTM1_CH1 == PTA9)
                {
                    PORTA_PCR9= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH1 == PTA13)
                {
                    PORTA_PCR13= PORT_PCR_MUX(3);
                }
                else if(FTM1_CH1 == PTB1)
                {
                    PORTB_PCR1= PORT_PCR_MUX(3);
                }
                break;

              default:
                return;
            }
            break;

          case FTM2:
            SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //使能FTM2时钟
            switch(ch)
            {
              case FTM_CH0:
                if(FTM2_CH0 == PTA10)
                {
                    PORTA_PCR10= PORT_PCR_MUX(3);
                }
                else if(FTM2_CH0 == PTB18)
                {
                    PORTB_PCR18= PORT_PCR_MUX(3);
                }
                break;

              case FTM_CH1:
                if(FTM2_CH1 == PTA11)
                {
                    PORTA_PCR11= PORT_PCR_MUX(3);
                }
                else if(FTM2_CH1 == PTB19)
                {
                    PORTB_PCR19= PORT_PCR_MUX(3);
                }
                break;

              default:
                return;
            }
            break;
          default:
            break;

            #ifdef MK60FX
      case FTM3:
        SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

        switch(ch)
        {
          case FTM_CH0:
            if(FTM3_CH0 == PTE5)
            {
                PORTE_PCR5=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH0 == PTD0)
            {
                PORTD_PCR0=PORT_PCR_MUX(4);
            }
            break;

          case FTM_CH1:
            if(FTM3_CH1 == PTE6)
            {
                PORTE_PCR6=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH1 == PTD1)
            {
                PORTD_PCR1=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH2:
            if(FTM3_CH2 == PTE7)
            {
                PORTE_PCR7=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH2 == PTD2)
            {
                PORTD_PCR2=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH3:
            if(FTM3_CH3 == PTE8)
            {
                PORTE_PCR8=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH3 == PTD3)
            {
                PORTD_PCR3=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH4:
            if(FTM3_CH4 == PTE9)
            {
                PORTE_PCR9=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH4 == PTC8)
            {
                PORTC_PCR8=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH5:
            if(FTM3_CH5 == PTE10)
            {
                PORTE_PCR10=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH5 == PTC9)
            {
                PORTC_PCR9=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH6:
            if(FTM3_CH6 == PTE11)
            {
                PORTE_PCR11=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH6 == PTC10)
            {
                PORTC_PCR10=PORT_PCR_MUX(4);
            }
            break;
          case FTM_CH7:
            if(FTM3_CH7 == PTE12)
            {
                PORTE_PCR12=PORT_PCR_MUX(6);
            }
            else if(FTM3_CH7 == PTC11)
            {
                PORTC_PCR11=PORT_PCR_MUX(4);
            }
            break;
          default:  return;

        }
#endif
        }

    }
    else
    {

        switch(ftmn)
        {

          case FTM1:
            SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //使能FTM1时钟
            if(FTM1_QDPHA == PTA8)                  //管脚复用
            {
                PORTA_PCR8= PORT_PCR_MUX(6);
            }
            else if(FTM1_QDPHA == PTA12)
            {
                PORTA_PCR12= PORT_PCR_MUX(7);
            }
            else if(FTM1_QDPHA == PTB0)
            {
                PORTB_PCR0= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }

            if(FTM1_QDPHB == PTA9)
            {
                PORTA_PCR9= PORT_PCR_MUX(6);
            }
            else if(FTM1_QDPHB == PTA13)
            {
                PORTA_PCR13= PORT_PCR_MUX(7);
            }
            else if(FTM1_QDPHB == PTB1)
            {
                PORTB_PCR1= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }
            break;


          case FTM2:
            SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //使能FTM2时钟
            if(FTM2_QDPHA == PTA10)                  //管脚复用
            {
                PORTA_PCR10= PORT_PCR_MUX(6);
            }
            else if(FTM2_QDPHA == PTB18)
            {
                PORTB_PCR18= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }

            if(FTM2_QDPHB == PTA11)                  //管脚复用
            {
                PORTA_PCR11= PORT_PCR_MUX(6);
            }
            else if(FTM2_QDPHB == PTB19)
            {
                PORTB_PCR19= PORT_PCR_MUX(6);
            }
            else
            {
                break;
            }break;
        }

    }
}

