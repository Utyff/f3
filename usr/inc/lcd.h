#ifndef F3_LCD_H
#define F3_LCD_H

#include "_main.h"

// st7793 has only vertical orientation
// set the x coordinate instruction
//#define LCD_SET_X 0x2a
#define LCD_SET_H 0x200
#define LCD_START_H 0x210
#define LCD_END_H 0x211
// Set the y coordinate instruction
//#define LCD_SET_Y 0x2b
#define LCD_SET_V 0x201
#define LCD_START_V 0x212
#define LCD_END_V 0x213
// begin to write the GRAM command
//#define LCD_WR_RAM_CMD 0x2c
#define LCD_WR_RAM_CMD 0x202 // st7793

#define MAX_X 400
#define MAX_Y 240

extern u16 POINT_COLOR; // Pen color
extern u16 BACK_COLOR;  // Background color


// Scan direction definition
#define L2R_U2D  0 // From left to right, top to bottom
#define L2R_D2U  1 // from left to right, from bottom to top
#define R2L_U2D  2 // Right to left, top to bottom
#define R2L_D2U  3 // Right to left, bottom to top

#define U2D_L2R  4 // from top to bottom, left to right
#define U2D_R2L  5 // from top to bottom, right to left
#define D2U_L2R  6 // From bottom to top, left to right
#define D2U_R2L  7 // From bottom to top, right to left

#define DFT_SCAN_DIR  L2R_U2D  // The default scan direction

// 16bit RGB colors
#define WHITE        0xFFFF
#define BLACK        0x0000
#define BLUE         0x001F
#define BRED         0XF81F
#define GRED         0XFFE0
#define GBLUE        0X07FF
#define RED          0xF800
#define MAGENTA      0xF81F
#define GREEN        0x07E0
#define CYAN         0x7FFF
#define YELLOW       0xFFE0
#define BROWN        0XBC40
#define BRRED        0XFC07  // reddish brown
#define GRAY         0X8430  // Gray  1000 0100 0011 0000

#define DARKGRAY     0X6208  // Gray  0100 0010 0000 1000
#define DARKBLUE     0X01CF
#define LIGHTBLUE    0X7D7C
#define GRAYBLUE     0X5458

#define LIGHTGREEN   0X841F
#define LIGHTGRAY    0XEF5B


#ifdef __cplusplus
extern "C" {
#endif

void LCD_Init(void);                                                        // Initialize
void LCD_Clear(u16 Color);                                                    // Clear the screen
void LCD_Clear8(u8 Color);                                                    // Clear the screen
void LCD_SetCursor(u16 Xpos, u16 Ypos);                                        // Set the cursor
void LCD_DrawPoint(u16 x, u16 y);                                            // Draw the points
void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color);                                // Quickly draw points
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);                                        // Draw a circle
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);                            // Draw lines
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);                        // Draw the rectangle
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color);                        // fill the monochrome
void LCD_drawBMP(u16 sx, u16 sy, u16 ex, u16 ey, const u16 *bmp);                // fill in the specified color
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode);                        // display a character
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);                        // display a number
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode);                // display numbers
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, const char *p, u8 mode);    // display a string,12/16 font
void LCD_Set_Window(u16 sx, u16 sy, u16 ex, u16 ey); // Set the window

#ifdef __cplusplus
}
#endif

#endif
