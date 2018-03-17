#include <_main.h>
#include <lcd.h>
#include "fmc_dma.h"


#define BYTE_IN_DOT 2
#define SCREEN_DOTS (320*24)
#define SCREEN_BYTES (SCREEN_DOTS*BYTE_IN_DOT)


uint16_t screenBuffer[SCREEN_DOTS] = {0,1,2};

void initScreenBuf() {
    for(int i=0; i<SCREEN_DOTS; i++) {
        screenBuffer[i]=MAGENTA;
    }
}

void clearScreen_dma(uint16_t color) {
    for(int i=0; i<SCREEN_DOTS; i++) {
        screenBuffer[i]=color;
    }
    copy2Screen_dma();
}

void copy2Screen_dma()
{
    LCD_SetCursor(0, 0);         // set the cursor position
    LCD_WriteRAM_Prepare();      // start writing GRAM

    // copy first half of screen
/*    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)screenBuffer, LCD->LCD_RAM, SCREEN_DOTS/2);
    HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1, HAL_DMA_FULL_TRANSFER, 1000);

    // copy second half of screen
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(screenBuffer+SCREEN_DOTS/2), LCD->LCD_RAM, SCREEN_DOTS/2);
    HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1, HAL_DMA_FULL_TRANSFER, 1000);//*/
}
