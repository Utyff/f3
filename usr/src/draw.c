#include <dwt.h>
#include "draw.h"
#include "graph.h"


void drawFrame() {
    uint16_t x, y;

    // erase graph
    drawGraph(BLACK);

    uint32_t drawTicks = DWT_Get_Current_Tick();

    // draw grid
    POINT_COLOR = WHITE;
    for (y = GRID_STEP + GRAPH_START_Y; y < (uint16_t) (GRAPH_START_Y + GRAHP_SIZE_Y); y += GRID_STEP) {
        for (x = GRID_STEP; x < GRAPH_SIZE_X; x += GRID_STEP) {
            LCD_DrawPoint(x, y);
        }
    }

    // draw cross scale
    LCD_Fill(0, GRAPH_START_Y + GRAHP_SIZE_Y / 2, GRAPH_SIZE_X - 1, GRAPH_START_Y + GRAHP_SIZE_Y / 2, GRAY);
    LCD_Fill(160, 0, 160, MAX_Y - 1, GRAY);

    LCD_Set_Window(0, 0, MAX_X - 1, MAX_Y - 1);

    // draw time
    drawTicks = DWT_Elapsed_Tick(drawTicks);
    POINT_COLOR = YELLOW;
    BACK_COLOR = BLACK;
    LCD_ShowxNum(130, 227, drawTicks / DWT_IN_MICROSEC, 6, 12, 8);
}

void drawScreen() {
    drawFrame();

    u32 t0 = DWT_Get_Current_Tick();

    buildGraph1ch();
    drawGraph(BLUE);

    // count time for one circle
    u32 ticks = DWT_Elapsed_Tick(t0);
    POINT_COLOR = YELLOW;
    LCD_ShowxNum(170, 227, ticks / DWT_IN_MICROSEC, 6, 12, 8);
}
