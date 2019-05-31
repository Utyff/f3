#include <_main.h>
#include <adc.h>
#include <delay.h>

#define MAX_SAMPLES 1024
uint32_t dataPoints12[MAX_SAMPLES];
uint32_t dataPoints34[MAX_SAMPLES];

/**
 * ADC1 channel 1 - PA0
 * ADC2 channel 2 - PA5
 */


void ADC_init2() {
    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock
//    RCC->AHBENR |= RCC_AHBENR_ADC34EN; // turn on ADC34 clock

    // disable the ADC. ADC conversation must be stopped.
//    ADC1->CR |= ADC_CR_ADDIS;
//    ADC2->CR |= ADC_CR_ADDIS;
//    ADC3->CR |= ADC_CR_ADDIS;
//    ADC4->CR |= ADC_CR_ADDIS;
    // wait when ADC disable
//    while (ADC1->CR & ADC_CR_ADEN);
//    while (ADC2->CR & ADC_CR_ADEN);

    // Set Prescaler ADC for Asynchronous clock mode
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1; // Prescaler ADC12 - PLL clock divided by 1
//    RCC->CFGR2 |= RCC_CFGR2_ADCPRE34_DIV1; // Prescaler ADC34 - PLL clock divided by 1

    // Set ADC clock
    // 00: (Asynchronous clock mode) PLL used
    // 01: HCLK/1 (Synchronous clock mode) AHB used
    // 10: HCLK/2
    // 11: HCLK/4
#define ADC_CCR_CLK (0b01u << ADC_CCR_CKMODE_Pos)
    ADC12_COMMON->CCR |= ADC_CCR_CLK;
//    ADC34_COMMON->CCR |= (0b01u << ADC_CCR_CKMODE_Pos);

    // enable the ADC voltage regulator
    ADC1->CR &= ~ADC_CR_ADVREGEN_1;
    ADC2->CR &= ~ADC_CR_ADVREGEN_1;
//    ADC3->CR &= ~(1u << 29u);
//    ADC4->CR &= ~(1u << 29u);

    ADC1->CR |= ADC_CR_ADVREGEN_0;
    ADC2->CR |= ADC_CR_ADVREGEN_0;
//    ADC3->CR |= (1u << 28u);
//    ADC4->CR |= (1u << 28u);

    // start ADC calibration cycle
    ADC1->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC1->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC2->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC2->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
//    ADC3->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
//    while (ADC3->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
//    ADC4->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
//    while (ADC4->CR & ADC_CR_ADCAL);

    // Delay for interleaved mode. Set only when ADEN=0
    // (SAMPLE_TIME + CONV. TIME) /2
    // (4.5 + 12.5) /2 = 8 tics
    ADC12_COMMON->CCR |= (0b0111u << ADC_CCR_DELAY_Pos);

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
//    ADC3->CR |= ADC_CR_ADEN;
//    ADC4->CR |= ADC_CR_ADEN;

    // wait till ADC start
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    while (!(ADC2->ISR & ADC_ISR_ADRDY));
//    while (!(ADC3->ISR & (1u << 0u)));
//    while (!(ADC4->ISR & (1u << 0u)));

    // Select ADC Channels
    ADC1->SQR1 = (1u << ADC_SQR1_SQ1_Pos); // 1-st channel for ADC1
    ADC2->SQR1 = (2u << ADC_SQR1_SQ1_Pos); // 2-d  channel for ADC2
//    ADC3->SQR1 = (1u << 6u);
//    ADC4->SQR1 = (3u << 6u);

    // Set sampling time for channels
    // 000: 1.5 ADC clock cycles
    // 001: 2.5 ADC clock cycles
    // 010: 4.5 ADC clock cycles
    // 011: 7.5 ADC clock cycles
    // 100: 19.5 ADC clock cycles
    // 101: 61.5 ADC clock cycles
    // 110: 181.5 ADC clock cycles
    // 111: 601.5 ADC clock cycles
#define SAMPLING_TIME  0b010u
    ADC1->SMPR1 |= (SAMPLING_TIME << ADC_SMPR1_SMP1_Pos); // ADC1 channel 1
    ADC2->SMPR1 |= (SAMPLING_TIME << ADC_SMPR1_SMP2_Pos); // ADC2 channel 2
//    ADC3->SMPR1 |= (0b000u << 3u);
//    ADC4->SMPR1 |= (0b000u << 3u);

    // Regular channel sequence length - 1
    ADC1->SQR1 |= (0b0000u << ADC_SQR1_L_Pos);
    ADC2->SQR1 |= (0b0000u << ADC_SQR1_L_Pos);
//    ADC3->SQR1 |= (0b0000u << 0u);
//    ADC4->SQR1 |= (0b0000u << 0u);

    // set data resolution
    // 00: 12-bit
    // 01: 10-bit
    // 10: 8-bit
    // 11: 6-bit
#define ADC_RESOLUTION (0b00u << ADC_CFGR_RES_Pos)
    ADC1->CFGR |= ADC_RESOLUTION;
    ADC2->CFGR |= ADC_RESOLUTION;
//    ADC3->CFGR |= 0b00u << ADC_CFGR_RES_Pos;
//    ADC4->CFGR |= 0b00u << ADC_CFGR_RES_Pos;

    // Enable continuous conversion mode. Only on the masters
    ADC1->CFGR |= ADC_CFGR_CONT; // Master ADC1 + ADC2
//    ADC3->CFGR |= (1u << 13u); // Master ADC3 + ADC4

    // dual mode - Regular simultaneous mode only
    // 00000: Independent mode
    // 00110: Regular simultaneous mode only
    // 00111: Interleaved mode only
    ADC12_COMMON->CCR |= (0b00111u << ADC_CCR_DUAL_Pos);
//    ADC34_COMMON->CCR |= (0b00110u << 0u);

    // DMA mode.  0 -> One Shot; 1 -> Circular
    // 0 - stop when DMA_CCR_TCIE
    ADC12_COMMON->CCR |= ADC_CCR_DMACFG;
//    ADC34_COMMON->CCR |= (0u << 13u);

    // DMA mode b10 - for 12-bit resolution
    ADC12_COMMON->CCR |= (0b10u << ADC_CCR_MDMA_Pos);
//    ADC34_COMMON->CCR |= (0b10u << 14u);

    // ADC start conversion
    ADC1->CR |= ADC_CR_ADSTART;
}

void DMA_init(void) {
    // Enable clocks DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//    RCC->AHBENR |= (1u << 1u); // DMA2

    // Transfer complete interrupt enable and Circular mode
    DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE | DMA_CCR_CIRC;
//    DMA2_Channel5->CCR |= (1u << 1u);

    // Memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
//    DMA2_Channel5->CCR |= (1u << 7u);

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << DMA_CCR_PSIZE_Pos);
//    DMA2_Channel5->CCR |= (0b11u << 8u);

    // Memory size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << DMA_CCR_MSIZE_Pos);
//    DMA2_Channel5->CCR |= (0b11u << 10u);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = MAX_SAMPLES;
//    DMA2_Channel5->CNDTR = MAX_SAMPLES;

    // Peripheral address register
    DMA1_Channel1->CPAR = (uint32_t) &ADC12_COMMON->CDR;
//    DMA2_Channel5->CPAR = (uint32_t) &ADC34_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR = (uint32_t) (&dataPoints12);
//    DMA2_Channel5->CMAR = (uint32_t) (&dataPoints34);

    // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
    DMA1->IFCR |= DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
//    DMA2->IFCR |= 0xFFu;

    // Enable DMA
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 CH1 interrupt
}

void ADC_init() {
    // Enable clocks GPIOA and  GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set mode b11 (analog input) for ADC pins
    GPIOA->MODER |= (0b11u << 0u); // PA0 for ADC1 ch1
    GPIOA->MODER |= (0b11u << 10u); // PA5 for ADC2 ch2
//    GPIOB->MODER |= (0b11u << 2u); // PB1 for ADC3 ch1
//    GPIOB->MODER |= (0b11u << 24u); // PB12 for ADC4 ch3

    DMA_init();
    ADC_init2();
}

uint16_t dmaG = 0;
uint16_t dmaT = 0;
uint16_t dmaH = 0;
uint16_t dmaE = 0;

void DMA1_Channel1_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
//        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        dmaT++;
    }
    if (DMA1->ISR & DMA_ISR_HTIF1) {
//        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        dmaH++;
    }
    if (DMA1->ISR & DMA_ISR_TEIF1) {
//        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        dmaE++;
    }
    if (DMA1->ISR & DMA_ISR_GIF1) {
        dmaG++;
    }
    DMA1->IFCR |= DMA_IFCR_CGIF1;
}
