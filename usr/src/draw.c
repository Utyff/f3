#include <dwt.h>
#include "draw.h"
#include "graph.h"


void drawFrame() {
    u16 x, y, step = 32;

    LCD_Clear(BLACK);
    POINT_COLOR = GRAY;  // Drawing pen color
    BACK_COLOR = BLACK;

    u32 t0 = DWT_Get_Current_Tick();

    for (y = step; y < MAX_Y; y += step) {
        if (y == 128) POINT_COLOR = GRAY;  // Drawing pen color
        else POINT_COLOR = DARKGRAY;
        LCD_Fill(0, y, MAX_X-1, y, POINT_COLOR);
    }

    for (x = step; x < MAX_X; x += step) {
        if (x == 160) POINT_COLOR = GRAY;  // Drawing pen color
        else POINT_COLOR = DARKGRAY;
        LCD_Fill(x, 0, x, MAX_Y-1, POINT_COLOR);
    }

    LCD_Set_Window(0,0,MAX_X-1,MAX_Y-1);

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    LCD_ShowxNum(130, 227, ticks / DWT_IN_MICROSEC, 8, 12, 9);
}

void drawScreen() {
    drawFrame();

    u32 t0 = DWT_Get_Current_Tick();

    drawGraph();

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    LCD_ShowxNum(170, 227, ticks / DWT_IN_MICROSEC, 8, 12, 9);
}
