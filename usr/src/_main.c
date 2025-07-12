#include <_main.h>
#include <delay.h>
#include <adc.h>



void mainInitialize() {
    DWT_Init();
    ADC_Init();
}

void mainCycle() {

    // starting conversions if it does not run
//    if (!(ADC1->CR & ADC_CR_ADSTART)) {
//        ADC_start();
//    }

    delay_ms(50);
}
