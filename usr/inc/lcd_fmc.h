#ifndef _LCD_FMC_H
#define _LCD_FMC_H

#include "_main.h"
#include <dwt.h>

//----------------- LCD port definition ----------------
// LCD address structure
typedef struct
{
    u16 LCD_REG;
    u16 LCD_RAM;
} LCD_TypeDef;

// Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4
// for A16 as command line for data command
//#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0001fffE))

// use Bank1.sector1 of NOR / SRAM, address bits HADDR [27,26]=00
// for A18 as command line for data command
// Note that the STM32 will shift to the right one bit when set!
#define LCD_BASE        ((u32)(0x60000000 | 0x00007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

__STATIC_INLINE void LCD_WR_REG(vu16 regval) {
    LCD->LCD_REG = regval;
#ifdef STM32F746xx
    SMALL_DELLAY++;
    SMALL_DELLAY++;
    SMALL_DELLAY++;
#endif
}

__STATIC_INLINE void LCD_WR_DATA(vu16 data) {
    LCD->LCD_RAM = data;
#ifdef STM32F746xx
    SMALL_DELLAY++;
    SMALL_DELLAY++;
    SMALL_DELLAY++;
#endif
}

// Read LCD data
// Return Value: Value read
__STATIC_INLINE u16 LCD_RD_DATA(void) {
#ifdef STM32F746xx
    SMALL_DELLAY++;
    SMALL_DELLAY++;
    SMALL_DELLAY++;
#endif
    return LCD->LCD_RAM;
}

// Write register
//LCD_Reg: Register Address
//LCD_RegValue: data to be written
__STATIC_INLINE void LCD_WriteReg(vu16 LCD_Reg, vu16 LCD_RegValue) {
    LCD_WR_REG(LCD_Reg);         // Write to write register number
    LCD_WR_DATA(LCD_RegValue);   // write data
}

// Read register
//LCD_Reg: Register Address
// Return Value: read data
__STATIC_INLINE u16 LCD_ReadReg(vu16 LCD_Reg) {
    LCD_WR_REG(LCD_Reg);        // Write the register number to be read
    return LCD_RD_DATA();       // Return value read
}


#endif //_LCD_FMC_H
