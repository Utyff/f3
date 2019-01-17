#include <dwt.h>
#include "draw.h"
#include "graph.h"


void drawFrame() {
    u16 x, y, step = 16;

//    LCD_Clear8(BLACK);
    eraseGraph();

    u32 t0 = DWT_Get_Current_Tick();

    POINT_COLOR = DARKGRAY;
    for (y = step; y < MAX_Y; y += step) {
        for(x = step; x < MAX_X; x += step) {
            LCD_DrawPoint(x,y);
        }
    }

    POINT_COLOR = GRAY;  // Drawing pen color
    LCD_Fill(0, 128, MAX_X-1, 128, POINT_COLOR);
    LCD_Fill(160, 0, 160, MAX_Y-1, POINT_COLOR);

    LCD_Set_Window(0,0,MAX_X-1,MAX_Y-1);

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    BACK_COLOR = BLACK;
    LCD_ShowxNum(130, 227, ticks / DWT_IN_MICROSEC, 6, 12, 8);
}

void drawScreen() {
    drawFrame();

    u32 t0 = DWT_Get_Current_Tick();

    drawGraph();

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    LCD_ShowxNum(170, 227, ticks / DWT_IN_MICROSEC, 6, 12, 8);
}
