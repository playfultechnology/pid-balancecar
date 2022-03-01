
#include "include.h"
#include "MK60_SYSTICK.h"
#define EACH_PER_MS    25   //每隔 25 ms 中断一次  systick 定时器是24位向下计数的定时器  最大装载值16777215 / 600 000 000= 0.2796 最大计时27ms

struct time{

  uint32_t fac_us;                  //us分频系数
  uint32_t fac_ms;                  //ms分频系数
  volatile uint32_t millisecond;    //ms
  uint64_t microsecond;             //us
  uint8_t ms_per_tick;              //1ms多少systick计数次数

}timer;
/**************************************************************************
*函数功能：初始化systick计数器
*入口参数：无
*返 回 值：无
*实    例：systime.init()
**************************************************************************/
static void systime_init()
{
	timer.fac_us = core_clk;
	timer.fac_ms = core_clk * 1000;
	timer.ms_per_tick = EACH_PER_MS;
    timer.millisecond = 100;
	SysTick_Config(timer.fac_ms * timer.ms_per_tick );   //开启systick中断
}

/**************************************************************************
*函数功能：systick  中断服务函数
*入口参数：无
*返 回 值：无
*实    例：无
**************************************************************************/
void SysTick_Handler(void)
{
	timer.millisecond += timer.ms_per_tick;
}

/**************************************************************************
*函数功能：systick  获取当前ms值
*入口参数：无
*返 回 值：当前ms值
*实    例：systime.get_time_ms()==计时功能  得到当前时间 ms
**************************************************************************/
static uint32_t get_current_time_ms(void)
{
    uint32_t val = SysTick->VAL;
	return timer.millisecond -  val/timer.fac_ms;
}

/**************************************************************************
*函数功能：systick  获取当前us值
*入口参数：无
*返 回 值：当前us值
*实    例：systime.get_time_us()==计时功能  得到当前时间 us
**************************************************************************/
static uint64_t get_current_time_us(void)
{
    uint32_t val = SysTick->VAL;
	return (uint64_t)((uint64_t)(timer.millisecond * 1000) -  val / timer.fac_us);
}

/**************************************************************************
*函数功能：systick  延时us
*入口参数：us:需要延时的us数
*返 回 值：无
*实    例：delay_us(10)==延时10us
**************************************************************************/
void delay_us( uint32_t us)
{
    uint64_t now = systime.get_time_us();
	uint64_t end_time = now + us - 1;
	while( systime.get_time_us() < end_time)
    {
        ;
    }
}

/**************************************************************************
*函数功能：systick  延时ms
*入口参数：us:需要延时的ms数
*返 回 值：无
*实    例：delay_ms(10)==延时10ms
**************************************************************************/
void delay_ms( uint16_t ms)
{
	systime.delay_us(ms * 1000);
}



systime_t  systime =
{
	systime_init,
	get_current_time_us,
	get_current_time_ms,
	delay_us,
	delay_ms
};



