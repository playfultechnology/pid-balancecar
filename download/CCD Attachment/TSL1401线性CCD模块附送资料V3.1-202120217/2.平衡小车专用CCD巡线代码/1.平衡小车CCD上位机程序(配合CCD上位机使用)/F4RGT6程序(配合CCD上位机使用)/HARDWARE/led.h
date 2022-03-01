#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		2     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小

/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOB
#define Buzzer_PIN GPIO_Pin_10
#define Buzzer PBout(10)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOE
#define LED_PIN GPIO_Pin_10
#define LED PEout(10) 
/*----------------------------------*/

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
extern int Led_Count;
#endif
