#ifndef F7_FMC_FMC_DMA_H
#define F7_FMC_FMC_DMA_H

#include <_main.h>

void initScreenBuf();
void clearScreen_dma(uint16_t color);
void copy2Screen_dma();

#endif //F7_FMC_FMC_DMA_H
