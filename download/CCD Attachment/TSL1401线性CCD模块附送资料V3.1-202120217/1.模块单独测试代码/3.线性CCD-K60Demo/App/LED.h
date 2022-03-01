#ifndef __LED_H
#define __LED_H


#define LED2  PTE28_OUT
#define LED3  PTA4_OUT

void LED_Init(void);//初始化LED

void Test_LED(void);//测试LED

void LED_OFF(uint8_t led);//关闭LED

void LED_ON(uint8_t led);//打开LED

void Led_Flash(uint8_t led,uint16_t time);//LED闪烁


#endif