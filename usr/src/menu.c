#include <lcd.h>
#include <keys.h>
#include <adc.h>
#include <generator.h>
#include <menu.h>


static void drawMenuADC() {
    POINT_COLOR = GREEN;
    LCD_ShowString(0, 0, 100, 100, 12, "ADC", 0x0);
    POINT_COLOR = WHITE;
    LCD_ShowChar(30, 0, 'd', 12, 0x0);
    LCD_ShowxNum(38, 0, (u32) rccAdcDivider, 3, 12, 0x0);
    LCD_ShowChar(68, 0, 'D', 12, 0x0);
    LCD_ShowxNum(76, 0, (u32) adcDelay, 3, 12, 0x0);
    LCD_ShowChar(106, 0, 's', 12, 0x0);
    LCD_ShowxNum(114, 0, (u32) sampleTime, 3, 12, 0x0);
}

static void drawMenuGEN() {
    POINT_COLOR = GREEN;
    LCD_ShowString(0, 0, 100, 100, 12, "GEN", 0x0);
    POINT_COLOR = WHITE;
    LCD_ShowChar(30, 0, 'p', 12, 0x0);
    LCD_ShowxNum(38, 0, (u32) tim1Prescaler, 5, 12, 0x0);
    LCD_ShowChar(73, 0, 'D', 12, 0x0);
    LCD_ShowxNum(81, 0, (u32) tim1Period, 5, 12, 0x0);
    LCD_ShowChar(116, 0, 's', 12, 0x0);
    LCD_ShowxNum(124, 0, (u32) tim1Pulse, 5, 12, 0x0);
    LCD_ShowChar(159, 0, 'f', 12, 0x0);
    LCD_ShowxNum(167, 0, (u32) tim1Freq, 7, 12, 0x0);
}

void drawMenu() {
    LCD_Fill(0,0,220,14,0);

    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;

    switch (keyMode) {
        case KEY_MODE_ADC:
            drawMenuADC();
            break;
        case KEY_MODE_GEN:
            drawMenuGEN();
            break;
        default:;
    }

    LCD_ShowxNum(0, 227, TIM8->CNT, 4, 12, 0x0);
    LCD_ShowxNum(30, 227, (u32) enc_count, 5, 12, 0x0);
    LCD_ShowxNum(0, 214, (u32) button1Count, 4, 12, 0x0);
    LCD_ShowChar(24, 214, 'b', 12, 0x0);
    LCD_ShowxNum(44, 214, (u32) sampleTime, 3, 12, 0x0);
    LCD_ShowChar(62, 214, 't', 12, 0x0);
    LCD_ShowxNum(78, 214, (u32) adcDelay, 3, 12, 0x0);
    LCD_ShowChar(96, 214, 'd', 12, 0x0);
    LCD_ShowxNum(110, 214, (u32) rccAdcDivider, 3, 12, 0x0);
    LCD_ShowChar(130, 214, 'r', 12, 0x0);
//    LCD_ShowxNum(180, 214, (u32) tim1Freq/100, 5, 12, 0x0);
//    LCD_ShowxNum(180, 214, (u32) ADCElapsedTick / DWT_IN_MICROSEC, 5, 12, 0x0);
}
