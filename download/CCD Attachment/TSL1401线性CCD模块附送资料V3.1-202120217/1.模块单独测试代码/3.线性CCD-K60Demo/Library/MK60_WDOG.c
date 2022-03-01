
#include "include.h"
#include "MK60_WDOG.h"	//包含wdog.h头文件

/**************************************************************************
*函数功能：关闭看门狗模块
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void WDOG_Disable(void)
{
  WDOG_Unlock();                                //给看门狗模块解锁，以便写寄存器
  WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;    //关闭看门狗
}

/**************************************************************************
*函数功能：使能看门狗模块
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void WDOG_Enable(void)
{
  WDOG_Unlock();                                //解锁看门狗，这样才能配置看门狗
  WDOG_STCTRLH |= WDOG_STCTRLH_WDOGEN_MASK;     //WDOGEN置位，使能 看门狗
}

/**************************************************************************
*函数功能：看门狗解锁
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void WDOG_Unlock(void)
{
  /* 注意: 不要单步调试此程序!!! ，否则会引起CPU复位*/
  DisableInterrupts;    //关总中断
  WDOG_UNLOCK = 0xC520; //写解锁寄存器
  WDOG_UNLOCK = 0xD928; //完成解锁
  EnableInterrupts;     //开总中断
}

/**************************************************************************
*函数功能：喂狗
*入口参数：无
*返 回 值：无
*实    例：无
*注    意:连续向此寄存器写入0xA602 、0xB480即可解锁
*         中间不得超过20个时钟周期，因此需要先关总中断
**************************************************************************/
void WDOG_Feed(void)
{
    //此函数不能单步执行
    uint8 tmp = __get_BASEPRI();    //用于返回寄存器 PRIMASK 的值(1bit)1表示关中断，0表示开中断
    DisableInterrupts;//关闭总中断，否则有可能没法在 20个周期内连续写入 WDOG_UNLOCK
    WDOG_REFRESH = 0xA602;//更新 看门狗（喂狗）
    WDOG_REFRESH = 0xB480;//更新 看门狗（喂狗）

    if(tmp == 0)
    {
        EnableInterrupts;
    }
}

/**************************************************************************
*函数功能：设置喂狗时间
*入口参数：无
*返 回 值：无
*实    例：WDOG_SetTime（10）设置喂狗时间10个时钟周期
**************************************************************************/
void WDOG_SetTime(uint32 ms)
{
    if(ms < 4)                                      //断言，计数时间最小为4个时钟周期（WDOG_TOVALL里说明）
        ms = 4;
    WDOG_Unlock();                                  //解锁看门狗，这样才能配置看门狗
    WDOG_PRESC = WDOG_PRESC_PRESCVAL(0);            //设置分频系数 = PRESCVAL +1(PRESCVAL取值范围为0~7)
    WDOG_TOVALH = ms >> 16;                         //设置喂狗时间
    WDOG_TOVALL = (uint16)ms;
    WDOG_STCTRLH = ( 0
                    | WDOG_STCTRLH_WDOGEN_MASK     //WDOGEN置位，使能 看门狗
                    //| WDOG_STCTRLH_CLKSRC_MASK   //看门狗时钟选择（0为 LDO ，1为bus 时钟）
                    | WDOG_STCTRLH_ALLOWUPDATE_MASK
                    | WDOG_STCTRLH_STOPEN_MASK
                    | WDOG_STCTRLH_WAITEN_MASK
                    | WDOG_STCTRLH_STNDBYEN_MASK
                    //|
                    );
}