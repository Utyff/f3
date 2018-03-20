#include "lcd.h"
#include "string.h"
#include "font.h"
#include "delay.h"
#include "gui.h"


void GUI_DrawPoint(u16 x, u16 y, u16 color) {
    LCD_SetCursor(x, y);
    LCD_DrawPoint_16Bit(color);
}

void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color) {
    u16 i, j;
    u16 width = ex - sx + 1;
    u16 height = ey - sy + 1;
    LCD_SetWindows(sx, sy, ex - 1, ey - 1);

#if LCD_USE8BIT_MODEL == 1
    LCD_RS_SET;
    LCD_CS_CLR;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            DATAOUT(color);
            LCD_WR_CLR;
            LCD_WR_SET;

            DATAOUT(color << 8);
            LCD_WR_CLR;
            LCD_WR_SET;
        }
    }
    LCD_CS_SET;
#else
    for(i=0;i<height;i++)
    {
        for(j=0;j<width;j++)
        LCD_WR_DATA(color);
    }
#endif
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
}


void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2) {
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)incx = 1;
    else if (delta_x == 0)incx = 0;
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0;
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)distance = delta_x;
    else distance = delta_y;
    for (t = 0; t <= distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}


void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2) {
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}


void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2) {
    LCD_Fill(x1, y1, x2, y2, POINT_COLOR);

}

void _draw_circle_8(int xc, int yc, int x, int y, u16 c) {
    GUI_DrawPoint(xc + x, yc + y, c);
    GUI_DrawPoint(xc - x, yc + y, c);
    GUI_DrawPoint(xc + x, yc - y, c);
    GUI_DrawPoint(xc - x, yc - y, c);
    GUI_DrawPoint(xc + y, yc + x, c);
    GUI_DrawPoint(xc - y, yc + x, c);
    GUI_DrawPoint(xc + y, yc - x, c);
    GUI_DrawPoint(xc - y, yc - x, c);
}


void gui_circle(int xc, int yc, u16 c, int r, int fill) {
    int x = 0, y = r, yi, d;
    d = 3 - 2 * r;
    if (fill) {
        while (x <= y) {
            for (yi = x; yi <= y; yi++)
                _draw_circle_8(xc, yc, x, yi, c);

            if (d < 0) {
                d = d + 4 * x + 6;
            } else {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    } else {
        while (x <= y) {
            _draw_circle_8(xc, yc, x, y, c);
            if (d < 0) {
                d = d + 4 * x + 6;
            } else {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
}


u32 mypow(u8 m, u8 n) {
    u32 result = 1;
    while (n--)result *= m;
    return result;
}

void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode) {
    u16 len = strlen((const char *) str);
    u16 x1 = (lcddev.width - len * 8) / 2;
    Show_Str(x + x1, y, fc, bc, str, size, mode);
}

/*******************************************************************************
����� BitMap ����������� ������� width � ������� height 16 ��� � ������� x,y.
*******************************************************************************/
void Gui_Drawbmp16(u16 x, u16 y, u16 width, u16 height, const unsigned char *p) {
    int i;
    unsigned char picH, picL;
    LCD_SetWindows(x, y, x + width - 1, y + height - 1);
    for (i = 0; i < width * height; i++) {
        picL = *(p + i * 2);
        picH = *(p + i * 2 + 1);
        LCD_DrawPoint_16Bit(picH << 8 | picL);
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
}


void LCD_DrawString(u16 x, u16 y, u16 FontColor, u16 BackColor, char *str, u8 mode, sFONT *font) {
    while (*str) {
        x = LCD_Putchar(x, y, FontColor, BackColor, *str++, mode, font);
    }
}

uint32_t LCD_Putchar(u16 x, u16 y, u16 FontColor, u16 BackColor, char c, u8 mode, sFONT *font) {
    uint32_t i, j;
    unsigned short Data;
    sFONT *CurrentFont = font;
    uint32_t offset = (c - 32) * CurrentFont->Height;
    uint16_t width = CurrentFont->Width;

    for (i = 0; i < CurrentFont->Height; i++) {
        Data = CurrentFont->table[offset + i];
        for (j = 0; j < width; j++) {
            if (CurrentFont != &Font8x16) {
                if ((((Data & ((0x80 << ((CurrentFont->Width / 12) * 8)) >> j)) == 0x00) &&
                     (CurrentFont->Width <= 12)) ||
                    (((Data & (0x1 << j)) == 0x00) && (CurrentFont->Width > 12))) {
                    if (mode) GUI_DrawPoint(x + j, y + i, BackColor);
                } else {
                    GUI_DrawPoint(x + j, y + i, FontColor);
                }
            } else {
                if ((Data >> j) & 0x01) {
                    GUI_DrawPoint(x + j, y + i, FontColor);
                } else {

                    if (mode) GUI_DrawPoint(x + j, y + i, BackColor);
                }
            }

        }
    }

    return x + width;
}
