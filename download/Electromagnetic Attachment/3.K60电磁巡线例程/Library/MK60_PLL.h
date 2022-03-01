
#ifndef __PLL_H__
#define __PLL_H__

extern u8 core_clk;//��λMHZ
extern u8 bus_clk;//��λMHZ

//PLL����
typedef enum clk_option
{
    PLLUSR      ,
    PLL48    = 48,
    PLL50    = 50,
    PLL96    = 96,
    PLL100   = 100,
    PLL110   = 110,
    PLL120   = 120,
    PLL125   = 125,
    PLL130   = 130,
    PLL140   = 140,
    PLL150   = 150,
    PLL160   = 160,
    PLL170   = 170,
    PLL180   = 180,
    PLL200   = 200, //�Ƽ�ʹ��ƵΪ200��180
    PLL225   = 225, //��Ƶ����Ƶ�ʣ�ϵͳ�����ȶ��������ܷɣ����ã�
    PLL250   = 250
} clk_option;



/*********************** PLL���ܺ��� **************************/
void PLL_Init(clk_option);     //���໷��ʼ��




#endif
