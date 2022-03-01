#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//����������Ҫ�õ���ͷ�ļ�

//The associated header file for the peripheral 
//��������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "adc.h"
#include "DataScope_DP.h"


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/
extern 	u8 SciBuf[200];  //�洢�ϴ�����λ������Ϣ
extern u16 ADV[128];
extern u8 CCD_Zhongzhi,CCD_Yuzhi;   //����CCD  ���

void systemInit(void);

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
