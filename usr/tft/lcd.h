#ifndef __LCD_H
#define __LCD_H

#include "_main.h"

/**************************************************************************************************
LCD_RST - PC5 changed !!!
LCD_RD - PC6
LCD_WR - PC7
LCD_RS - PC8
LCD_CS - PC9

LCD_D0 - PB8
...
LCD_D8 - PB15
**************************************************************************************************/

typedef struct {
    u16 width;
    u16 height;
    u16 id;
    u8  dir;
    u16 wramcmd;
    u16 setxcmd;
    u16 setycmd;
} _lcd_dev;

extern _lcd_dev lcddev;

#define USE_HORIZONTAL          1
#define LCD_USE8BIT_MODEL       1

#if USE_HORIZONTAL == 1
#define LCD_W 320
#define LCD_H 240
#else
#define LCD_W 240
#define LCD_H 320
#endif

extern u16 POINT_COLOR;
extern u16 BACK_COLOR;


#define CTL_PORT  GPIOA
#define DATA_PORT GPIOB

#define LCD_RST_PIN     5
#define LCD_CS_PIN      9
#define LCD_RS_PIN      8
#define LCD_WR_PIN      7
#define LCD_RD_PIN      6

#define LCD_CS_SET   CTL_PORT->BSRR=1<<LCD_CS_PIN    //PC9
#define LCD_RS_SET   CTL_PORT->BSRR=1<<LCD_RS_PIN    //PC8
#define LCD_WR_SET   CTL_PORT->BSRR=1<<LCD_WR_PIN    //PC7
#define LCD_RD_SET   CTL_PORT->BSRR=1<<LCD_RD_PIN    //PC6
#define LCD_RST_SET  CTL_PORT->BSRR=1<<LCD_RST_PIN    //PC5

#define LCD_RST_CLR  CTL_PORT->BRR=1<<LCD_RST_PIN    //PC5
#define LCD_CS_CLR   CTL_PORT->BRR=1<<LCD_CS_PIN     //PC9
#define LCD_RS_CLR   CTL_PORT->BRR=1<<LCD_RS_PIN     //PC8
#define LCD_WR_CLR   CTL_PORT->BRR=1<<LCD_WR_PIN     //PC7
//#define LCD_RD_CLR   CTL_PORT->BRR=1<<LCD_RD_PIN     //PC6

#define DATAOUT(x) DATA_PORT->ODR=x;
//#define DATAIN   DATA_PORT->IDR;


#define L2R_U2D  0
#define L2R_D2U  1
#define R2L_U2D  2
#define R2L_D2U  3

#define U2D_L2R  4
#define U2D_R2L  5
#define D2U_L2R  6
#define D2U_R2L  7

#define DFT_SCAN_DIR  L2R_U2D

#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define BRED        0XF81F
#define GRED        0XFFE0
#define GBLUE       0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN       0XBC40
#define BRRED       0XFC07
#define GRAY        0X8430


extern u16 BACK_COLOR, POINT_COLOR;

void LCD_Init(void);


void LCD_Clear(u16 Color);

void LCD_SetCursor(u16 Xpos, u16 Ypos);

void LCD_DrawPoint(u16 x, u16 y);

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);

void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd);

void LCD_DrawPoint_16Bit(u16 color);

void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);

void LCD_WR_DATA(u16 data);

void LCD_WriteRAM_Prepare(void);

void LCD_WriteRAM(u16 RGB_Code);

void LCD_SetParam(void);

/*
#if LCD_USE8BIT_MODEL==1//
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	DATAOUT(data<<8);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	}
	#else//
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	}
#endif
*/

#endif
