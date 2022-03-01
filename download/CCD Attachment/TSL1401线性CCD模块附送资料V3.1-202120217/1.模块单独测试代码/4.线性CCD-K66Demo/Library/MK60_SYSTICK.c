
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
【功能说明】初始化systick计数器
【参    数】无
【返 回 值】无
【例    子】systime.init();开启systick定时器
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
【功能说明】systick  中断服务函数
【返 回 值】无
【参    数】无
【例    子】
**************************************************************************/
void SysTick_Handler(void)
{
	timer.millisecond += timer.ms_per_tick;
}

/**************************************************************************
【功能说明】systick  获取当前ms值
【参    数】无
【返 回 值】当前ms值
【例    子】systime.get_time_ms();计时功能  得到当前时间 ms
**************************************************************************/
static uint32_t get_current_time_ms(void)
{
    uint32_t val = SysTick->VAL;
	return timer.millisecond -  val/timer.fac_ms;
}
/**************************************************************************
【功能说明】systick  获取当前ms值
【参    数】无
【返 回 值】当前ms值
【例    子】systime.get_time_us();计时功能  得到当前时间 us
**************************************************************************/
static uint64_t get_current_time_us(void)
{
    uint32_t val = SysTick->VAL;
	return (uint64_t)((uint64_t)(timer.millisecond * 1000) -  val / timer.fac_us);
}

/**************************************************************************
【功能说明】systick  延时us
【参    数】无
【返 回 值】无
【例    子】systime.delay_us(1000000);延时功能  延时1s
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
【功能说明】systick  延时ms
【参    数】无
【返 回 值】无
【例    子】systime.delay_ms(1000);延时功能 延时1s
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



