#ifndef __SYSTICK_H
#define __SYSTICK_H
#include "common.h"
typedef struct
{
	void (* init) (void);
	uint64_t (* get_time_us) (void);
	uint32_t (* get_time_ms) (void);
	void (* delay_us)(uint32_t);
	void (* delay_ms)(uint16_t);
}systime_t;
extern systime_t  systime;

void delay_us( uint32_t us);//us—” ±
void delay_ms( uint16_t ms);//ms—” ±


#endif



