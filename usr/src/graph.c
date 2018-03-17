#include <graph.h>
#include <dwt.h>
#include <DataBuffer.h>


/**
 * Make and draw oscillogram
 */

uint8_t graph[MAX_X];
float scaleX = 1;  // no more then 1

/**
 * Looking for trigger event position in 1 channel samples array
 * @return if trigger found - index of start element. Other case - 0
 */
int triggerStart1ch(u8 const *samples) {
    int i;
    u8 trgLvl = 128;
    u8 trgRdy = 0;

    for (i = 0; i < BUF_SIZE / 2; i++) {
        if (trgRdy == 0) {
            if (samples[i] < trgLvl)
                trgRdy = 1;
            continue;
        }

        if (samples[i] > trgLvl)
            return i;
    }
    return 0;
}


// start position in buffer
// number of samples to display

uint32_t BuildGraphTick;

/**
 * Build graph for 1 channels samples array
 */

void buildGraph1ch() {
    uint32_t t0 = DWT_Get_Current_Tick();
    int i, j;
    float x;

    u8 *samples = samplesBuffer;
    if (firstHalf != 0) samples += BUF_SIZE / 2;

    x = 0;
    j = -1;
    i = triggerStart1ch(samples);
    for (; i < BUF_SIZE / 2; i++) {
        if ((int) x != j) {
            j = (int) x;
            if (j >= MAX_X) break;
            graph[j] = samples[i];
        } else {
            graph[j] = (graph[j] + samples[i]) >> 1; // arithmetical mean
        }
        x += scaleX;
    }
    BuildGraphTick = DWT_Elapsed_Tick(t0);
}

uint32_t DrawGraphTick;

void drawGraph() {
    u8 prev;

    buildGraph1ch();
    uint32_t t0 = DWT_Get_Current_Tick();

    POINT_COLOR = BLUE;
    prev = graph[0];
    for (u16 i = 1; i < MAX_X; i++) {
        //LCD_DrawLine(i - (u16) 1, prev, i, graph[i]);
        LCD_Fill(i , prev, i, graph[i], POINT_COLOR);
        prev = graph[i];
    }
    LCD_Set_Window(0,0,MAX_X-1,MAX_Y-1);

    DrawGraphTick = DWT_Elapsed_Tick(t0);
//  LCD_ShowxNum(150,227, DrawGraphTick/168,  10,12, 9);
//  LCD_ShowxNum(190,227, BuildGraphTick/168, 10,12, 9);
} //*/
