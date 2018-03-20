#include "lcd.h"
#include "delay.h"

// https://github.com/iwalpola/Adafruit_ILI9341_8bit_STM

_lcd_dev lcddev;

u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
//u16 DeviceCode;

void LCD_Init_CMD();

/*****************************************************************/
void LCD_WR_REG(u8 data) {
#if LCD_USE8BIT_MODEL == 1
    LCD_RS_CLR;
    LCD_CS_CLR;
    DATAOUT(data << 8);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;
#else
    LCD_RS_CLR;
    LCD_CS_CLR;
    DATAOUT(data);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;
#endif
}

/*****************************************************************/
void LCD_WR_DATA(u16 data) {
#if LCD_USE8BIT_MODEL == 1
    LCD_RS_SET;
    LCD_CS_CLR;
    DATAOUT(data << 8);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;

#else
    LCD_RS_SET;
    LCD_CS_CLR;
    DATAOUT(data);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;
#endif
}

/*****************************************************************/
void LCD_DrawPoint_16Bit(u16 color) {
#if LCD_USE8BIT_MODEL == 1
    LCD_CS_CLR;
    LCD_RD_SET;
    LCD_RS_SET;
    DATAOUT(color);
    LCD_WR_CLR;
    LCD_WR_SET;
    DATAOUT(color << 8);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;
#else
    LCD_WR_DATA(color);
#endif
}

/*****************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue) {
    LCD_WR_REG(LCD_Reg);
    LCD_WR_DATA(LCD_RegValue);
}

/*****************************************************************/
void LCD_WriteRAM_Prepare(void) {
    LCD_WR_REG(lcddev.wramcmd);
}

/*****************************************************************/
void LCD_DrawPoint(u16 x, u16 y) {
    LCD_SetCursor(x, y);
#if LCD_USE8BIT_MODEL == 1
    LCD_CS_CLR;
    LCD_RD_SET;
    LCD_RS_SET;
    DATAOUT(POINT_COLOR);
    LCD_WR_CLR;
    LCD_WR_SET;
    DATAOUT(POINT_COLOR << 8);
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_CS_SET;
#else
    LCD_WR_DATA(POINT_COLOR);
#endif
}

/*****************************************************************/
void LCD_Clear(u16 Color) {
    u32 index = 0;
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
#if LCD_USE8BIT_MODEL == 1
    LCD_RS_SET;
    LCD_CS_CLR;
    for (index = 0; index < lcddev.width * lcddev.height; index++) {
        DATAOUT(Color);
        LCD_WR_CLR;
        LCD_WR_SET;

        DATAOUT(Color << 8);
        LCD_WR_CLR;
        LCD_WR_SET;
    }
    LCD_CS_SET;
#else
    for(index=0;index<lcddev.width*lcddev.height;index++)
    {
        LCD_WR_DATA(Color);
    }
#endif
}

#define GPIO_SetBits(port, pins) port->BSRR = pins
#define GPIO_Pin_All 0xFF

/*****************************************************************/
void LCD_GPIOInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

/* 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = 1<<LCD_CS_PIN|1<<LCD_RS_PIN|1<<LCD_WR_PIN|1<<LCD_RD_PIN|1<<LCD_RST_PIN;	   //GPIO_Pin_10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); //GPIOC  */
    GPIO_SetBits(CTL_PORT, 1 << LCD_CS_PIN | 1 << LCD_RS_PIN | 1 << LCD_WR_PIN | 1 << LCD_RD_PIN | 1 << LCD_RST_PIN);

/*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //GPIOB */
    GPIO_SetBits(DATA_PORT, GPIO_Pin_All);
}

/*****************************************************************/
void LCD_RESET(void) {
    LCD_RST_CLR;
    delay_ms(100);
    LCD_RST_SET;
    delay_ms(50);
}

/*****************************************************************/
void LCD_Init(void) {
    LCD_GPIOInit();
    LCD_RESET();

    LCD_Init_CMD();

    LCD_SetParam();
    //LCD_Clear(BLUE);
}


/*****************************************************************/
void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd) {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(xStar >> 8);
    LCD_WR_DATA(0x00FF & xStar);
    LCD_WR_DATA(xEnd >> 8);
    LCD_WR_DATA(0x00FF & xEnd);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(yStar >> 8);
    LCD_WR_DATA(0x00FF & yStar);
    LCD_WR_DATA(yEnd >> 8);
    LCD_WR_DATA(0x00FF & yEnd);
    LCD_WriteRAM_Prepare();
}

/*****************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos) {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(Xpos >> 8);
    LCD_WR_DATA(0x00FF & Xpos);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(Ypos >> 8);
    LCD_WR_DATA(0x00FF & Ypos);
    LCD_WriteRAM_Prepare();
}

/*****************************************************************/
void LCD_SetParam(void) {
    lcddev.wramcmd = 0x2C;
#if USE_HORIZONTAL == 1
    lcddev.dir = 1;
    lcddev.width = 320;
    lcddev.height = 240;
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;
    LCD_WriteReg(0x36, 0x6C);
#else
    lcddev.dir=0;
    lcddev.width=240;
    lcddev.height=320;
    lcddev.setxcmd=0x2A;
    lcddev.setycmd=0x2B;
    LCD_WriteReg(0x36,0xC9);
#endif
}	


void LCD_Init_CMD() {
    LCD_WR_REG(0x01);//Software Reset
    DWT_Delay_us(1);
    LCD_WR_REG(0xCB);//Power Control A
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x02);
    DWT_Delay_us(1);
    LCD_WR_REG(0xCF);//Power Control B
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xC1);
    LCD_WR_DATA(0x30);
    DWT_Delay_us(1);
    LCD_WR_REG(0xE8);//Driver timing control A
    LCD_WR_DATA(0x85);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x78);
    DWT_Delay_us(1);
    LCD_WR_REG(0xEA);//Driver timing control B
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    DWT_Delay_us(1);
    LCD_WR_REG(0xED);//Power on Sequence control
    LCD_WR_DATA(0x64);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0x12);
    LCD_WR_DATA(0x81);
    DWT_Delay_us(1);
    LCD_WR_REG(0xF7);//Pump ratio control
    LCD_WR_DATA(0x20);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC0);//Power Control 1
    LCD_WR_DATA(0x10);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC1);//Power Control 2
    LCD_WR_DATA(0x10);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC5);//VCOM Control 1
    LCD_WR_DATA(0x3E);
    LCD_WR_DATA(0x28);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC7);//VCOM Control 2
    LCD_WR_DATA(0x86);
    DWT_Delay_us(1);
//    TFT9341_SetRotation(0);
    DWT_Delay_us(1);
    LCD_WR_REG(0x3A);//Pixel Format Set
    LCD_WR_DATA(0x55);//16bit
    DWT_Delay_us(1);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x18);// Частота кадров 79 Гц
    DWT_Delay_us(1);
    LCD_WR_REG(0xB6);//Display Function Control
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x82);
    LCD_WR_DATA(0x27);//320 строк
    DWT_Delay_us(1);
    LCD_WR_REG(0xF2);//Enable 3G (пока не знаю что это за режим)
    LCD_WR_DATA(0x00);//не включаем
    DWT_Delay_us(1);
    LCD_WR_REG(0x26);//Gamma set
    LCD_WR_DATA(0x01);//Gamma Curve (G2.2) (Кривая цветовой гаммы)
    DWT_Delay_us(1);
    LCD_WR_REG(0xE0);//Positive Gamma  Correction
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x31);
    LCD_WR_DATA(0x2B);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x4E);
    LCD_WR_DATA(0xF1);
    LCD_WR_DATA(0x37);
    LCD_WR_DATA(0x07);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x09);
    LCD_WR_DATA(0x00);
    DWT_Delay_us(1);
    LCD_WR_REG(0xE1);//Negative Gamma  Correction
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x14);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0x11);
    LCD_WR_DATA(0x07);
    LCD_WR_DATA(0x31);
    LCD_WR_DATA(0xC1);
    LCD_WR_DATA(0x48);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x31);
    LCD_WR_DATA(0x36);
    LCD_WR_DATA(0x0F);
    DWT_Delay_us(1);
    LCD_WR_REG(0x11);//Выйдем из спящего режим
    HAL_Delay(150);
    LCD_WR_REG(0x29);//Включение дисплея
    LCD_WR_DATA(0x2C);
    HAL_Delay(150);
}

void LCD_Init_CMD2 () {
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0xCF);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xC1);
    LCD_WR_DATA(0X30);
    LCD_WR_REG(0xED);
    LCD_WR_DATA(0x64);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0X12);
    LCD_WR_DATA(0X81);
    LCD_WR_REG(0xE8);
    LCD_WR_DATA(0x85);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x7A);
    LCD_WR_REG(0xCB);
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x02);
    LCD_WR_REG(0xF7);
    LCD_WR_DATA(0x20);
    LCD_WR_REG(0xEA);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0xC0);    //Power control
    LCD_WR_DATA(0x1B);   //VRH[5:0]
    LCD_WR_REG(0xC1);    //Power control
    LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0]
    LCD_WR_REG(0xC5);    //VCM control
    LCD_WR_DATA(0x30);     //3F
    LCD_WR_DATA(0x30);     //3C
    LCD_WR_REG(0xC7);    //VCM control2
    LCD_WR_DATA(0XB7);
    LCD_WR_REG(0x36);    // Memory Access Control
    LCD_WR_DATA(0x48);
    LCD_WR_REG(0x3A);
    LCD_WR_DATA(0x55);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1A);
    LCD_WR_REG(0xB6);    // Display Function Control
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0xA2);
    LCD_WR_REG(0xF2);    // 3Gamma Function Disable
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0x26);    //Gamma curve selected
    LCD_WR_DATA(0x01);
    LCD_WR_REG(0xE0);    //Set Gamma
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x2A);
    LCD_WR_DATA(0x28);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x54);
    LCD_WR_DATA(0XA9);
    LCD_WR_DATA(0x43);
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0XE1);    //Set Gamma
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x15);
    LCD_WR_DATA(0x17);
    LCD_WR_DATA(0x07);
    LCD_WR_DATA(0x11);
    LCD_WR_DATA(0x06);
    LCD_WR_DATA(0x2B);
    LCD_WR_DATA(0x56);
    LCD_WR_DATA(0x3C);
    LCD_WR_DATA(0x05);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x0F);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x3f);
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xef);
    LCD_WR_REG(0x11); //Exit Sleep
    delay_ms(120);
    LCD_WR_REG(0x29); //display on
}
