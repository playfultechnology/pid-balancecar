#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件

//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "adc.h"
#include "DataScope_DP.h"


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern 	u8 SciBuf[200];  //存储上传到上位机的信息
extern u16 ADV[128];
extern u8 CCD_Zhongzhi,CCD_Yuzhi;   //线性CCD  相关

void systemInit(void);

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
