#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PBin(14)	
#define MODE PBin(12)	//ģʽ����
#define IR1 PAin(8)  	//����1
#define IR2 PAin(11)	//����2
void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);
u8  select(void);

#endif 
