#include <graph.h>
#include <dwt.h>
#include <DataBuffer.h>


/**
 * Make and draw oscillogram
 */

static uint8_t graph[MAX_X];
static int startGraph;
static int endGraph;
float scaleX = 1;  // no more than 1
float scaleY = 0.64f;
uint8_t trgLvl = 0x3f;

/**
 * Looking for trigger event position in 1 channel samples array
 * @return if trigger found - index of start element. Other case - 0
 */
int triggerStart1ch(u8 const *samples) {
    int i1 = BUF_SIZE / 2;
    int i2 = BUF_SIZE / 2 + 1;
    uint8_t trg1Rdy = samples[i2] > trgLvl;
    uint8_t trg2Rdy = samples[i1] < trgLvl;

    while (i1 != 0) {
        // looking left side from middle
        if (trg1Rdy == 0) {
            if (samples[i1] > trgLvl) {
                trg1Rdy = 1;
            }
        } else if (samples[i1] < trgLvl) {
            return i1;
        }

        // looking right side from middle
        if (trg2Rdy == 0) {
            if (samples[i2] < trgLvl) {
                trg2Rdy = 1;
            }
        } else if (samples[i2] > trgLvl) {
            return i2;
        }

        i1--;
        i2++;
    }
    return 0;
}


uint32_t buildGraphTick;

/**
 * Build graph for 1 channels samples array
 */

void buildGraph1ch() {
    uint32_t t0 = DWT_Get_Current_Tick();
    int i, j;
    float x;

    // graph my start not from x=0
    i = triggerStart1ch(samplesBuffer) - (int) (GRAPH_SIZE_X * scaleX / 2);
    if (i < 0) {
        j = (int) (scaleX * (float) (0 - i));
        i = 0;
    } else {
        j = 0;
    }
    startGraph = j;
    x = (float) j;
    j--;

    for (; i < BUF_SIZE; i++) {
        register uint8_t val = (uint8_t) ((float) samplesBuffer[i] * scaleY);
        if ((int) x != j) {
            j = (int) x;
            if (j >= GRAPH_SIZE_X) break;
            graph[j] = val;
        } else {
            graph[j] = (graph[j] + val) >> 1; // arithmetical mean
        }
        x += scaleX;
    }
    endGraph = j;
    buildGraphTick = DWT_Elapsed_Tick(t0);
}

void drawGraph(uint16_t color) {
    u8 prev;

    prev = graph[startGraph] + GRAPH_START_Y;
    for (int i = startGraph + 1; i < endGraph; i++) {
        //LCD_DrawLine(i - (u16) 1, prev, i, graph[i]);
        LCD_Fill(i, prev, i, graph[i] + GRAPH_START_Y, color);
        prev = graph[i] + GRAPH_START_Y;
    }
    LCD_Set_Window(0, 0, MAX_X - 1, MAX_Y - 1);
}
