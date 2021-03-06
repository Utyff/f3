#ifndef _LCD_GPIO_H
#define _LCD_GPIO_H

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

#define USE_HORIZONTAL          1

#if USE_HORIZONTAL == 1
#define LCD_W 320
#define LCD_H 240
#else
#define LCD_W 240
#define LCD_H 320
#endif

#define CTL_PORT  GPIOA
#define DATA_PORT GPIOB

#define LCD_RST_PIN     0
#define LCD_CS_PIN      1
#define LCD_RS_PIN      2
#define LCD_WR_PIN      3
#define LCD_RD_PIN      4

#ifdef  LCD_RST_Pin
#define LCD_RST_SET  CTL_PORT->BSRR=1<<LCD_RST_PIN
#define LCD_RST_CLR  CTL_PORT->BRR=1<<LCD_RST_PIN
#else
#define LCD_RST_SET
#define LCD_RST_CLR
#endif

#define LCD_CS_SET   CTL_PORT->BSRR=1<<LCD_CS_PIN  // disable LCD bus
#define LCD_RS_SET   CTL_PORT->BSRR=1<<LCD_RS_PIN  // select data
#define LCD_WR_SET   CTL_PORT->BSRR=1<<LCD_WR_PIN
#define LCD_RD_SET   CTL_PORT->BSRR=1<<LCD_RD_PIN

#define LCD_CS_CLR   CTL_PORT->BRR=1<<LCD_CS_PIN  // enable LCD bus
#define LCD_RS_CLR   CTL_PORT->BRR=1<<LCD_RS_PIN  // select register
#define LCD_WR_CLR   CTL_PORT->BRR=1<<LCD_WR_PIN
#define LCD_RD_CLR   CTL_PORT->BRR=1<<LCD_RD_PIN

#define DATAOUT(x) { *(volatile uint8_t *)&DATA_PORT->ODR = (x); }
// !!! Data must be 8bit !!! The highest bits must be 0.
//#define DATAOUT(x) { DATA_PORT->BSRR = ((~(x))<<16) | (x); }
//#define DATAOUT(x) DATA_PORT->ODR=x;
//#define DATAIN   DATA_PORT->IDR;
//__attribute__( ( always_inline ) ) __STATIC_INLINE void DATAOUT(u8 data) {
//    DATA_PORT->BSRR = ((~(data))<<16) | (data);
//}

// write without control RS
#define LCD_WR_DATA8_SHORT(data) { DATAOUT(data); LCD_WR_CLR; LCD_WR_SET; }
#define LCD_WR_DATA16_SHORT(data)  { LCD_WR_DATA8_SHORT((data) >> 8);  LCD_WR_DATA8_SHORT(data); }

#define LCD_WR_REG(data) { \
    LCD_RS_CLR; \
    DATAOUT(data); \
    LCD_WR_CLR; \
    LCD_WR_SET; \
}

#define LCD_WR_DATA8(data) { \
    LCD_RS_SET; \
    DATAOUT(data); \
    LCD_WR_CLR; \
    LCD_WR_SET; \
}

#define LCD_WR_DATA16(data) { \
    LCD_RS_SET; \
    DATAOUT(data >> 8); \
    LCD_WR_CLR; \
    LCD_WR_SET; \
    DATAOUT(data); \
    LCD_WR_CLR; \
    LCD_WR_SET; \
}

/*
__STATIC_INLINE u8 LCD_RD_DATA8(void) {
#ifdef LCD_READ_SUPPORT
    LCD_RD_SET;

    LCD_RD_CLR;
#endif
}
*/

// Write register
//LCD_Reg: Register Address
//LCD_RegValue: data to be written
__attribute__( ( always_inline ) ) __STATIC_INLINE void LCD_WriteReg(vu16 LCD_Reg, vu16 LCD_RegValue) {
    LCD_WR_REG(LCD_Reg);         // Write to write register number
    LCD_WR_DATA8(LCD_RegValue);  // write data
}

#endif //_LCD_GPIO_H
