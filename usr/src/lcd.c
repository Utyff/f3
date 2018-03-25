#include "lcd.h"
#include "delay.h"
#include "font.h"

// https://github.com/iwalpola/Adafruit_ILI9341_8bit_STM

_lcd_dev lcddev;

u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;


void LCD_Init_CMD();



void LCD_DrawPoint(u16 x, u16 y) {
    LCD_SetCursor(x, y);
    LCD_WR_DATA(POINT_COLOR);
}

void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color) {
    LCD_SetCursor(x, y);
    LCD_WR_DATA(color);
}


void LCD_Clear(u16 Color) {
    LCD_Set_Window(0, 0, lcddev.width - (u16)1, lcddev.height - (u16)1);
#ifdef LCD_USE8BIT_MODEL
    LCD_RS_SET;

    u32 total =  lcddev.width * lcddev.height;
    for (int i = 0; i < total; i++) {
        LCD_WR_DATA8_SHORT(Color>>8);
        LCD_WR_DATA8_SHORT(Color);
    }
#endif
}

#define GPIO_SetBits(port, pins) port->BSRR = pins
#define GPIO_Pin_All 0xFF

void LCD_GPIOInit(void) {
/*    GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
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
    LCD_CS_CLR;
}

void LCD_SetParam();

void LCD_Init(void) {
    LCD_GPIOInit();
    LCD_RESET();

    LCD_Init_CMD();

    LCD_SetParam();
    LCD_Clear(BLACK);
}


void LCD_Set_Window(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd) {
    LCD_WR_REG(LCD_SET_X);
    LCD_WR_DATA8((u8)(xStar >> 8));
    LCD_WR_DATA8((u8)(0x00FF & xStar));
    LCD_WR_DATA8((u8)(xEnd >> 8));
    LCD_WR_DATA8((u8)(0x00FF & xEnd));
    LCD_WR_REG(LCD_SET_Y);
    LCD_WR_DATA8((u8)(yStar >> 8));
    LCD_WR_DATA8((u8)(0x00FF & yStar));
    LCD_WR_DATA8((u8)(yEnd >> 8));
    LCD_WR_DATA8((u8)(0x00FF & yEnd));
    LCD_WriteRAM_Prepare();
}

void LCD_SetCursor(u16 x, u16 y) {
    LCD_WR_REG((u8)(LCD_SET_X));
    LCD_WR_DATA8((u8)(x >> 8));
    LCD_WR_DATA8((u8)(0x00FF & x));
    LCD_WR_REG((u8)(LCD_SET_Y));
    LCD_WR_DATA8((u8)(y >> 8));
    LCD_WR_DATA8((u8)(0x00FF & y));
    LCD_WriteRAM_Prepare();
}

/*****************************************************************/
void LCD_SetParam(void) {
#if USE_HORIZONTAL == 1
    lcddev.dir = 1;
    lcddev.width = 320;
    lcddev.height = 240;
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0x6C);
#else
    lcddev.dir=0;
    lcddev.width=240;
    lcddev.height=320;
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0xC9);
#endif
}	


void LCD_Init_CMD() {
    LCD_WR_REG(0x01);//Software Reset
    DWT_Delay_us(1);
    LCD_WR_REG(0xCB);//Power Control A
    LCD_WR_DATA8(0x39);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x34);
    LCD_WR_DATA8(0x02);
    DWT_Delay_us(1);
    LCD_WR_REG(0xCF);//Power Control B
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xC1);
    LCD_WR_DATA8(0x30);
    DWT_Delay_us(1);
    LCD_WR_REG(0xE8);//Driver timing control A
    LCD_WR_DATA8(0x85);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x78);
    DWT_Delay_us(1);
    LCD_WR_REG(0xEA);//Driver timing control B
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    DWT_Delay_us(1);
    LCD_WR_REG(0xED);//Power on Sequence control
    LCD_WR_DATA8(0x64);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x81);
    DWT_Delay_us(1);
    LCD_WR_REG(0xF7);//Pump ratio control
    LCD_WR_DATA8(0x20);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC0);//Power Control 1
    LCD_WR_DATA8(0x10);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC1);//Power Control 2
    LCD_WR_DATA8(0x10);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC5);//VCOM Control 1
    LCD_WR_DATA8(0x3E);
    LCD_WR_DATA8(0x28);
    DWT_Delay_us(1);
    LCD_WR_REG(0xC7);//VCOM Control 2
    LCD_WR_DATA8(0x86);
    DWT_Delay_us(1);
//    TFT9341_SetRotation(0);
    DWT_Delay_us(1);
    LCD_WR_REG(0x3A);//Pixel Format Set
    LCD_WR_DATA8(0x55);//16bit
    DWT_Delay_us(1);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x18);// Частота кадров 79 Гц
    DWT_Delay_us(1);
    LCD_WR_REG(0xB6);//Display Function Control
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x82);
    LCD_WR_DATA8(0x27);//320 строк
    DWT_Delay_us(1);
    LCD_WR_REG(0xF2);//Enable 3G (пока не знаю что это за режим)
    LCD_WR_DATA8(0x00);//не включаем
    DWT_Delay_us(1);
    LCD_WR_REG(0x26);//Gamma set
    LCD_WR_DATA8(0x01);//Gamma Curve (G2.2) (Кривая цветовой гаммы)
    DWT_Delay_us(1);
    LCD_WR_REG(0xE0);//Positive Gamma  Correction
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x2B);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x4E);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x00);
    DWT_Delay_us(1);
    LCD_WR_REG(0xE1);//Negative Gamma  Correction
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0xC1);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x0F);
    DWT_Delay_us(1);
    LCD_WR_REG(0x11);//Выйдем из спящего режим
    HAL_Delay(150);
    LCD_WR_REG(0x29);//Включение дисплея
    LCD_WR_DATA8(0x2C);
    HAL_Delay(150);
}

// m^n function
// Return value:m^n-th power.
u32 LCD_Pow(u8 m, u8 n) {
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

// Display a character in the specified location
//x,y: Start coordinates
//num:characters to be displayed:" "--->"~"
//size: Font size 12/16/24
//mode: the superposition mode (1) or non-overlapping mode (0)
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode) {
    u8 temp, t1, t;
    u16 y0 = y;
    // get a font character set corresponding to the number of bytes occupied by a dot
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
    // Setup Window
    num = num - ' ';// values obtained after offset
    for (t = 0; t < csize; t++) {
        if (size == 12)temp = asc2_1206[num][t];        // call 1206 font
        else if (size == 16)temp = asc2_1608[num][t];    // call 1608 font
        else if (size == 24)temp = asc2_2412[num][t];    // call 2412 font
        else return;                                // no fonts
        for (t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)LCD_Fast_DrawPoint(x, y, POINT_COLOR);
            else if (mode == 0)LCD_Fast_DrawPoint(x, y, BACK_COLOR);
            temp <<= 1;
            y++;
            if (y >= lcddev.height)return;        // over the region
            if ((y - y0) == size) {
                y = y0;
                x++;
                if (x >= lcddev.width)return;    // over the region
                break;
            }
        }
    }
}

// Show figures, the high is 0, or show
//x,y: the starting point coordinates
//num: Numerical (0~999999999);
//len: length (ie the number of digits to be displayed)
//size: Font Size
//mode:
//[7]:0, no padding;1, filled with 0.
//[6:1]: Reserved
//[0]:0, non-superimposition display;1, superimposed display.
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode) {
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                if (mode & 0X80)LCD_ShowChar(x + (size / (u16)2) * t, y, '0', size, mode & (u8)0X01);
                else            LCD_ShowChar(x + (size / (u16)2) * t, y, ' ', size, mode & (u8)0X01);
                continue;
            } else enshow = 1;
        }
        LCD_ShowChar(x + (size / (u16)2) * t, y, temp + (u8)'0', size, mode & (u8)0X01);
    }
}
