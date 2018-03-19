#ifndef F3_LCD_GPIO_H
#define F3_LCD_GPIO_H

#include <_main.h>
#include "delay.h"

#define LCD_DATA_PORT LCD1_GPIO_Port
#define LCD_RST_PORT LCD_RST_GPIO_Port
#define LCD_CS_PORT LCD_CS_GPIO_Port
#define LCD_RS_PORT LCD_RS_GPIO_Port
#define LCD_WR_PORT LCD_WR_GPIO_Port
#define LCD_RD_PORT LCD_RD_GPIO_Port


#define SET_LCD_RST     LCD_RST_PORT->BSRR = LCD_RST_Pin
#define RESET_LCD_RST   LCD_RST_PORT->BRR = LCD_RST_Pin
#define SET_LCD_CS      LCD_CS_PORT->BSRR = LCD_CS_Pin   // enable LCD controller. Always set.
//#define RESET_LCD_CS    LCD_CS_PORT->BRR = LCD_CS_Pin
#define SET_LCD_RS      LCD_RS_PORT->BSRR = LCD_RS_Pin
#define RESET_LCD_RS    LCD_RS_PORT->BRR = LCD_RS_Pin    // read/write registers
#define SET_LCD_WR      LCD_WR_PORT->BSRR = LCD_WR_Pin   // read/write data
#define RESET_LCD_WR    LCD_WR_PORT->BRR = LCD_WR_Pin
#define SET_LCD_RD      LCD_RD_PORT->BSRR = LCD_RD_Pin
#define RESET_LCD_RD    LCD_RD_PORT->BRR = LCD_RD_Pin

__STATIC_INLINE void LCD_Reset(void) {
    SET_LCD_CS; // always set
    RESET_LCD_RST;
    delay_ms(5);
    SET_LCD_RST;
    delay_ms(5);
}

__STATIC_INLINE void LCD_WR_REG(uint8_t reg) {
    LCD_DATA_PORT->ODR = reg;
    RESET_LCD_RS;
    RESET_LCD_WR;
    SET_LCD_WR;
}

__STATIC_INLINE void LCD_WR_DATA(uint8_t data) {
    LCD_DATA_PORT->ODR = data;
    SET_LCD_RS;
    RESET_LCD_WR;
    SET_LCD_WR;
}

__STATIC_INLINE u8 LCD_RD_DATA(void) {
    RESET_LCD_RS;
    RESET_LCD_RD;
    uint8_t data = (uint8_t) LCD_DATA_PORT->ODR;
    SET_LCD_RD;
    return data;
}


// Write register
//LCD_Reg: Register Address
//LCD_RegValue: data to be written
__STATIC_INLINE void LCD_WriteReg(vu8 LCD_Reg, vu8 LCD_RegValue) {
    LCD_WR_REG(LCD_Reg);         // Write to write register number
    LCD_WR_DATA(LCD_RegValue);   // write data
}

// Read register
//LCD_Reg: Register Address
// Return Value: read data
__STATIC_INLINE u16 LCD_ReadReg(vu8 LCD_Reg) {
    LCD_WR_REG(LCD_Reg);        // Write the register number to be read
    return LCD_RD_DATA();       // Return value read
}


#endif //F3_LCD_GPIO_H
