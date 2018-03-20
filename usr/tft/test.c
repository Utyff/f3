#include "lcd.h"
#include "delay.h"
#include "gui.h"
#include "test.h"

u16 ColorTab[5] = {BRED, YELLOW, RED, GREEN, BLUE};

void DrawTestPage(u8 *str) {
    LCD_Fill(0, 0, lcddev.width, 20, BLUE);
    LCD_Fill(0, lcddev.height - 20, lcddev.width, lcddev.height, BLUE);
    POINT_COLOR = WHITE;
    Gui_StrCenter(0, 2, WHITE, BLUE, str, 16, 1);
    Gui_StrCenter(0, lcddev.height - 18, WHITE, BLUE, "EUGENEMCU.NICHOST.RU", 16, 1);//?������������?
    LCD_Fill(0, 20, lcddev.width, lcddev.height - 20, BLACK);
}


void main_test(void) {
    DrawTestPage("TFT LCD PROJECT");

    Gui_StrCenter(0, 100, WHITE, BLUE, "STM32F103RC, 8-BIT GPIO INTERFACE", 32, 1);//?������������?
    Gui_StrCenter(0, 120, YELLOW, BLUE, "2.4\" ILI9341 240X320", 16, 1);//?������������?

    delay_ms(3000);
}


void Test_FillRec(void) {
    u8 i = 0;
    DrawTestPage("TEST2:RECTANGLES");
    LCD_Fill(0, 20, lcddev.width, lcddev.height - 20, WHITE);
    for (i = 0; i < 5; i++) {
        LCD_DrawRectangle(lcddev.width / 2 - 80 + (i * 15), lcddev.height / 2 - 80 + (i * 15),
                          lcddev.width / 2 - 80 + (i * 15) + 60, lcddev.height / 2 - 80 + (i * 15) + 60);
        POINT_COLOR = ColorTab[i];
    }
    delay_ms(1500);
    LCD_Fill(0, 20, lcddev.width, lcddev.height - 20, WHITE);
    for (i = 0; i < 5; i++) {
        LCD_DrawFillRectangle(lcddev.width / 2 - 80 + (i * 15), lcddev.height / 2 - 80 + (i * 15),
                              lcddev.width / 2 - 80 + (i * 15) + 60, lcddev.height / 2 - 80 + (i * 15) + 60);
        POINT_COLOR = ColorTab[i];
    }
    delay_ms(1500);
}


void Test_Circle(void) {
    u8 i = 0;
    DrawTestPage("TEST3:CIRCLES");
    LCD_Fill(0, 20, lcddev.width, lcddev.height - 20, WHITE);
    for (i = 0; i < 5; i++)
        gui_circle(lcddev.width / 2 - 80 + (i * 25), lcddev.height / 2 - 50 + (i * 25), ColorTab[i], 30, 0);
    delay_ms(1500);
    LCD_Fill(0, 20, lcddev.width, lcddev.height - 20, WHITE);
    for (i = 0; i < 5; i++)
        gui_circle(lcddev.width / 2 - 80 + (i * 25), lcddev.height / 2 - 50 + (i * 25), ColorTab[i], 30, 1);
    delay_ms(1500);
}

void English_Font_test(void) {
    DrawTestPage("TEST4:ENGLISH FONT");
    POINT_COLOR = YELLOW;
    BACK_COLOR = BLUE;
    LCD_ShowString(10, 60, 12, "6X12:abcdefghijklmnopqrstuvwxyz0123456789", 0);
    LCD_ShowString(10, 75, 12, "6X12:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", 1);
    LCD_ShowString(10, 90, 12, "6X12:~!@#$%^&*()_+{}:<>?/|-+.", 0);
    LCD_ShowString(10, 110, 16, "8X16:abcdefghijklmnopqrstuvwxyz0123456789", 0);
    LCD_ShowString(10, 130, 16, "8X16:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", 1);
    LCD_ShowString(10, 150, 16, "8X16:~!@#$%^&*()_+{}:<>?/|-+.", 0);
    delay_ms(1200);
}





