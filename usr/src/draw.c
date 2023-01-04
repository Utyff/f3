#include <dwt.h>
#include "draw.h"
#include "graph.h"

#define GRID_STEP 16

void drawFrame() {
    u16 x, y;

//    LCD_Clear8(BLACK);
    eraseGraph();

    u32 t0 = DWT_Get_Current_Tick();

    POINT_COLOR = DARKGRAY;
    for (y = GRID_STEP; y < MAX_Y; y += GRID_STEP) {
        for (x = GRID_STEP; x < MAX_X; x += GRID_STEP) {
            LCD_DrawPoint(x, y);
        }
    }

    POINT_COLOR = GRAY;  // Drawing pen color
    LCD_Fill(0, 128, MAX_X - 1, 128, POINT_COLOR);
    LCD_Fill(160, 0, 160, MAX_Y - 1, POINT_COLOR);

    LCD_Set_Window(0, 0, MAX_X - 1, MAX_Y - 1);

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    BACK_COLOR = BLACK;
    LCD_ShowxNum(130, 227, ticks / DWT_IN_MICROSEC, 6, 12, 8);
}

void drawScreen() {
    drawFrame();

    u32 t0 = DWT_Get_Current_Tick();

//    drawGraph();

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    LCD_ShowxNum(170, 227, ticks / DWT_IN_MICROSEC, 6, 12, 8);
}
