#ifndef __LED_H
#define __LED_H


#define LED2  PTE28_OUT
#define LED3  PTA4_OUT

void LED_Init(void);//��ʼ��LED

void Test_LED(void);//����LED

void LED_OFF(uint8_t led);//�ر�LED

void LED_ON(uint8_t led);//��LED

void Led_Flash(uint8_t led,uint16_t time);//LED��˸


#endif