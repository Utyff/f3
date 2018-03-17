#ifndef __GRAPH_H
#define __GRAPH_H

#include "_main.h"
#include "lcd.h"


#define MAX_X 320
#define MAX_Y 240

extern float scaleX;
extern uint8_t graph[];

#ifdef __cplusplus
extern "C" {
#endif

void drawGraph();

#ifdef __cplusplus
}
#endif

#endif /* __GRAPH_H */
