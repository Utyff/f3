#include <lcd_gpio.h>
#include <lcd.h>
#include "font.h"
#include "delay.h"

/**
 * TFT LCD driver
 * Support driver IC models: ILI9341
 */

u16 POINT_COLOR = 0x0000; // Drawing pen color
u16 BACK_COLOR = 0xFFFF;  // background color

// Management LCD important parameters
_lcd_dev lcddev;


void LCD_Init_sequence();

// Start writing GRAM
__attribute__( ( always_inline ) ) __STATIC_INLINE void LCD_WriteRAM_Prepare(void) {
#pragma GCC diagnostic ignored "-Woverflow"
    LCD_WR_REG16(LCD_WR_RAM_CMD);
#pragma GCC diagnostic warning "-Woverflow"
}

// Set the cursor position
//Xpos: abscissa
//Ypos: ordinate
void LCD_SetCursor(u16 x, u16 y) {
#pragma GCC diagnostic ignored "-Woverflow"
    LCD_WR_REG16(LCD_SET_X);
    LCD_WR_DATA16(x);
    LCD_WR_REG16(LCD_SET_Y);
    LCD_WR_DATA16(y);
#pragma GCC diagnostic warning "-Woverflow"
}

// Set the window, and automatically sets the upper left corner of the window to draw point coordinates (sx,sy).
//sx,sy: window start coordinate (upper left corner)
//width,height: width and height of the window, must be greater than 0!!
// Form size:width*height.
void LCD_Set_Window(u16 sx, u16 sy, u16 ex, u16 ey) {
#pragma GCC diagnostic ignored "-Woverflow"
    LCD_WR_REG16(LCD_START_X);
    LCD_WR_DATA16(sx);
    LCD_WR_REG16(LCD_END_X);
    LCD_WR_DATA16(ex);

    LCD_WR_REG16(LCD_START_Y);
    LCD_WR_DATA16(sy);
    LCD_WR_REG16(LCD_END_Y);
    LCD_WR_DATA16(ey);

    LCD_WR_REG16(LCD_SET_X);
    LCD_WR_DATA16(sx);
    LCD_WR_REG16(LCD_SET_Y);
    LCD_WR_DATA16(sy);
#pragma GCC diagnostic warning "-Woverflow"
}

// Draw points
//x,y: coordinates
//POINT_COLOR: the color of this point
void LCD_DrawPoint(u16 x, u16 y) {
    if (x >= MAX_X || y >= MAX_Y)
        return;

    LCD_SetCursor(x, y);       // Set the cursor position
    LCD_WriteRAM_Prepare();    // Start writing GRAM
    LCD_WR_DATA16(POINT_COLOR);
}

// Draw the point fast
//x,y: coordinates
//color: color
void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color) {
    if (x >= MAX_X || y >= MAX_Y)
        return;

    LCD_SetCursor(x, y);
    LCD_WriteRAM_Prepare();    // Start writing GRAM
    LCD_WR_DATA16(color);
}


#define GPIO_SetBits(port, pins) port->BSRR = pins
#define GPIO_Pin_All 0xFF

void LCD_GPIOInit(void) {
    // deactivate WR and RD signals
    GPIO_SetBits(CTL_PORT, 1 << LCD_RS_PIN | 1 << LCD_WR_PIN | 1 << LCD_RD_PIN);
//    GPIO_SetBits(DATA_PORT, GPIO_Pin_All);

    LCD_CS_CLR; // Chip-select always active

#ifdef  LCD_RST_Pin
    LCD_RST_CLR;
    delay_ms(100);
    LCD_RST_SET;
    delay_ms(50);
#endif
}

void LCD_SetParam();

void LCD_Init(void) {
    LCD_GPIOInit();

    LCD_Init_sequence();

    LCD_SetParam();
    LCD_Clear(BLACK);
}


void LCD_SetParam(void) {
#if USE_HORIZONTAL == 1
    lcddev.dir = 1;
    lcddev.width = MAX_X;
    lcddev.height = MAX_Y;
//    LCD_WR_REG16(0x003);
//    LCD_WR_DATA16(0x1034);
#else
    lcddev.dir=0;
    lcddev.width=240;
    lcddev.height=320;
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0xC9);
#endif
}


void LCD_Init_sequence() {
#pragma GCC diagnostic ignored "-Woverflow"
    //st7793
    LCD_WR_REG16(0x0001);LCD_WR_DATA16(0x0100); // Driver Output Control, draw direction.
    LCD_WR_REG16(0x0003);LCD_WR_DATA16(0x1030); // 9030
    LCD_WR_REG16(0x0008);LCD_WR_DATA16(0x0808);
    LCD_WR_REG16(0x0090);LCD_WR_DATA16(0x8000);
    LCD_WR_REG16(0x0400);LCD_WR_DATA16(0x6200); // Base Image Number of Line. Set GS: (1<<15) |
    LCD_WR_REG16(0x0401);LCD_WR_DATA16(0x0001); // Base Image Display Control
    //-----------------------------------End Display Control setting-----------------------------------------//
    //-------------------------------- Power Control Registers Initial --------------------------------------//
    LCD_WR_REG16(0x00ff);LCD_WR_DATA16(0x0001);
    LCD_WR_REG16(0x0102);LCD_WR_DATA16(0x01b0);
    LCD_WR_REG16(0x0710);LCD_WR_DATA16(0x0016);
    LCD_WR_REG16(0x0712);LCD_WR_DATA16(0x000f);
    LCD_WR_REG16(0x0752);LCD_WR_DATA16(0x002f);
    LCD_WR_REG16(0x0724);LCD_WR_DATA16(0x001a);
    LCD_WR_REG16(0x0754);LCD_WR_DATA16(0x002a);
    //---------------------------------End Power Control Registers Initial -------------------------------//
    DWT_Delay_ms(100);
    //----------------------------------Display Windows 240 X 400----------------------------------------//
    LCD_WR_REG16(0x0210);LCD_WR_DATA16(0x0000);
    LCD_WR_REG16(0x0211);LCD_WR_DATA16(0x00ef);
    LCD_WR_REG16(0x0212);LCD_WR_DATA16(0x0000);
    LCD_WR_REG16(0x0213);LCD_WR_DATA16(0x018f);
    //----------------------------------End Display Windows 240 X 400----------------------------------//
    DWT_Delay_ms(10);
    //-------------------------------------Gamma Cluster Setting-------------------------------------------//
    LCD_WR_REG16(0x0380);LCD_WR_DATA16(0x0000);
    LCD_WR_REG16(0x0381);LCD_WR_DATA16(0x5f10);
    LCD_WR_REG16(0x0382);LCD_WR_DATA16(0x0b02);
    LCD_WR_REG16(0x0383);LCD_WR_DATA16(0x0614);
    LCD_WR_REG16(0x0384);LCD_WR_DATA16(0x0111);
    LCD_WR_REG16(0x0385);LCD_WR_DATA16(0x0000);
    LCD_WR_REG16(0x0386);LCD_WR_DATA16(0xa90b);
    LCD_WR_REG16(0x0387);LCD_WR_DATA16(0x0606);
    LCD_WR_REG16(0x0388);LCD_WR_DATA16(0x0612);
    LCD_WR_REG16(0x0389);LCD_WR_DATA16(0x0111);
    //---------------------------------------End Gamma Setting---------------------------------------------//
    //---------------------------------------Vcom Setting---------------------------------------------//
    LCD_WR_REG16(0x0702);LCD_WR_DATA16(0x003b);
    LCD_WR_REG16(0x00ff);LCD_WR_DATA16(0x0000);
    //---------------------------------------End Vcom Setting---------------------------------------------//
    LCD_WR_REG16(0x0007);LCD_WR_DATA16(0x0100);
    DWT_Delay_ms(200); //Delay 200ms
    LCD_WR_REG16(0x0200);LCD_WR_DATA16(0x0000);
    LCD_WR_REG16(0x0201);LCD_WR_DATA16(0x0000);
#pragma GCC diagnostic warning "-Woverflow"
}

// Clear screen function
//color: To clear the screen fill color
void LCD_Clear(u16 color) {
    LCD_Set_Window(0, 0, MAX_X - 1, MAX_Y - 1); // set the cursor position
    LCD_WriteRAM_Prepare();                 // start writing GRAM

    LCD_RS_SET;
    u32 totalPoints = lcddev.width * lcddev.height;  // get the total number of points

//    u32 t0 = DWT_Get_Current_Tick();
    for (u32 i = 0; i < totalPoints; i++) {
        LCD_WR_DATA16_SHORT(color);
    }
//    t0 = DWT_Elapsed_Tick(t0);
//    POINT_COLOR = WHITE;
//    BACK_COLOR = BLACK;
//    LCD_ShowxNum(100, 227, t0 / DWT_IN_MICROSEC, 8, 12, 8); // LCD_Clear - 45935 us
}

void LCD_Clear8(u8 color) {
    LCD_Set_Window(0, 0, MAX_X - 1, MAX_Y - 1); // set the cursor position
    LCD_WriteRAM_Prepare();                 // start writing GRAM

    LCD_RS_SET;
    u32 totalPoints = (u32) (lcddev.width * lcddev.height / 8);  // get the total number of points. 2 points in cycle

    DATAOUT(color);
//    u32 t0 = DWT_Get_Current_Tick();
    for (u32 i = 0; i < totalPoints; i++) {
        LCD_WR_CLR; LCD_WR_SET; // 1 point - 2 pulses
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;

        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;

        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;

        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
        LCD_WR_CLR; LCD_WR_SET;
    }
//    t0 = DWT_Elapsed_Tick(t0);

//    POINT_COLOR = WHITE;
//    BACK_COLOR = BLACK;
//    LCD_ShowxNum(100, 227, t0 / DWT_IN_MICROSEC, 8, 12, 8); // LCD_Clear - 45935 us
}

// Fill a single color in the designated area
//(sx,sy),(ex,ey): filled rectangle coordinates diagonal, area size:(ex-sx+1)*(ey-sy+1)
//color: To fill color
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color) {
    u16 tmp;
    if (sy > ey) {
        tmp = sy;
        sy = ey;
        ey = tmp;
    }
    u32 totalPoints = (ex - sx + (u16) 1) * (ey - sy + (u16) 1);

    LCD_Set_Window(sx, sy, ex, ey);          // set the cursor position
    LCD_WriteRAM_Prepare();                  // start writing GRAM
    LCD_RS_SET;
    for (int j = 0; j < totalPoints; j++) {  // display colors
        LCD_WR_DATA16_SHORT(color);
    }
}

// In the designated area to fill the specified color block
//(sx,sy),(ex,ey): filled rectangle coordinates diagonal, area size:(ex-sx+1)*(ey-sy+1)
//bmp: pointer to bmp array
void LCD_drawBMP(u16 sx, u16 sy, u16 ex, u16 ey, const u16 *bmp) {
    u16 height, width;
    u16 i, j;
    width = ex - sx + (u16) 1;            // get filled width
    height = ey - sy + (u16) 1;           // height
    for (i = 0; i < height; i++) {
        LCD_SetCursor(sx, sy + i);    // set the cursor position
        LCD_WriteRAM_Prepare();       // start writing GRAM
        LCD_RS_SET;
        for (j = 0; j < width; j++) { // write data
            LCD_WR_DATA16_SHORT(bmp[i * width + j]);
        }
    }
}

// Draw a line
//x1,y1: starting point coordinates
//x2,y2: end coordinates
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2) {
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // calculate the coordinates increment
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)incx = 1; // set the single-step directions
    else if (delta_x == 0)incx = 0;// vertical line
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0;// horizontal
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)distance = delta_x; // Select the basic incremental axis
    else distance = delta_y;
    for (t = 0; t <= distance + 1; t++)  // draw a line output
    {
        LCD_DrawPoint(uRow, uCol);       // draw points
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

// Draw a rectangle
//(x1,y1),(x2,y2): rectangle coordinates diagonal
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2) {
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}

// A circle the size of the appointed position Videos
//(x,y): the center
//r    : Radius
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r) {
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);             // determine the next point position sign
    while (a <= b) {
        LCD_DrawPoint(x0 + a, y0 - b);             //5
        LCD_DrawPoint(x0 + b, y0 - a);             //0
        LCD_DrawPoint(x0 + b, y0 + a);             //4
        LCD_DrawPoint(x0 + a, y0 + b);             //6
        LCD_DrawPoint(x0 - a, y0 + b);             //1
        LCD_DrawPoint(x0 - b, y0 + a);
        LCD_DrawPoint(x0 - a, y0 - b);             //2
        LCD_DrawPoint(x0 - b, y0 - a);             //7
        a++;
        // Use Bresenham algorithm Circle
        if (di < 0)di += 4 * a + 6;
        else {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
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

// m^n function
// Return value:m^n-th power.
u32 LCD_Pow(u8 m, u8 n) {
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

// Show figures, the high is 0, no display
//x,y : the starting point coordinates
//len : Digits
//size: Font Size
//color: color
//num: Numerical(0~4294967295);
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size) {
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + (size / 2) * t, y, ' ', size, 0);
                continue;
            } else enshow = 1;
        }
        LCD_ShowChar(x + (size / 2) * t, y, temp + '0', size, 0);
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
                if (mode & 0X80)LCD_ShowChar(x + (size / (u16) 2) * t, y, '0', size, mode & (u8) 0X01);
                else LCD_ShowChar(x + (size / (u16) 2) * t, y, ' ', size, mode & (u8) 0X01);
                continue;
            } else enshow = 1;
        }
        LCD_ShowChar(x + (size / (u16) 2) * t, y, temp + (u8) '0', size, mode & (u8) 0X01);
    }
}

// Display string
//x,y: the starting point coordinates
//width,height: size of the area
//size: Font Size
//*p: string starting address
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, const char *p, u8 mode) {
    u16 x0 = x;
    width += x;
    height += y;
    while ((*p <= '~') && (*p >= ' '))// judgment is not illegal characters!
    {
        if (x >= width) {
            x = x0;
            y += size;
        }
        if (y >= height)break;//Exit
        LCD_ShowChar(x, y, *p, size, mode);
        x += size / 2;
        p++;
    }
}
