#ifndef __GUI_H__
#define __GUI_H__

#include "font.h"

extern sFONT Font16x24;
extern sFONT Font12x12;
extern sFONT Font8x12;
extern sFONT Font8x8;
extern sFONT Font8x16;

void GUI_DrawPoint(u16 x, u16 y, u16 color);

void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color);

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);

void Draw_Circle(u16 x0, u16 y0, u16 fc, u8 r);

void LCD_ShowChar(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode);

void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);

void LCD_Show2Num(u16 x, u16 y, u16 num, u8 len, u8 size, u8 mode);

void LCD_ShowString(u16 x, u16 y, u8 size, u8 *p, u8 mode);

void Show_Str(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode);

void Gui_Drawbmp16(u16, u16, u16, u16, const unsigned char *);

void gui_circle(int xc, int yc, u16 c, int r, int fill);

void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode);

void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2);

void LCD_DrawString(u16 x, u16 y, u16 FontColor, u16 BackColor, char *str, u8 mode, sFONT *font);

uint32_t LCD_Putchar(u16 x, u16 y, u16 FontColor, u16 BackColor, char c, u8 mode, sFONT *font);

#endif

