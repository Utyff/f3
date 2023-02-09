#ifndef F3_GRAPH_H
#define F3_GRAPH_H

#include "_main.h"
#include "lcd.h"

#define GRID_STEP 16u
#define GRAPH_SIZE_X 320u
#define GRAHP_SIZE_Y 192u
#define GRAPH_START_X 0u
#define GRAPH_START_Y ((MAX_Y - GRAHP_SIZE_Y) / 2u)

extern float scaleX;

#ifdef __cplusplus
extern "C" {
#endif

void buildGraph1ch();
void drawGraph(uint16_t color);
//void eraseGraph();

#ifdef __cplusplus
}
#endif

#endif /* F3_GRAPH_H */
