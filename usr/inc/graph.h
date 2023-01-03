#ifndef F3_GRAPH_H
#define F3_GRAPH_H

#include "_main.h"
#include "lcd.h"


extern float scaleX;
extern uint8_t graph[];

#ifdef __cplusplus
extern "C" {
#endif

void drawGraph();
void eraseGraph();

#ifdef __cplusplus
}
#endif

#endif /* F3_GRAPH_H */
