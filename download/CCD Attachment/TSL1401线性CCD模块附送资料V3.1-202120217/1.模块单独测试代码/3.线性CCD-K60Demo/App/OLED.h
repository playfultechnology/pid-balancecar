#ifndef __OLED_H
#define __OLED_H


//OLED¿ØÖÆÓÃº¯Êý
void OLED_WR_Byte(u8 dat,u8 cmd);

void OLED_Display_On(void);

void OLED_Display_Off(void);

void OLED_Refresh_Gram(void);

void OLED_Init(void);

void OLED_Clear(void);

void OLED_DrawPoint(u8 x,u8 y,u8 t);

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);

void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);

void OLED_ShowString(u8 x,u8 y,const u8 *p);

#endif

