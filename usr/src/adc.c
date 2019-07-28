#include <_main.h>
#include "adc.h"
#include "delay.h"


uint16_t dmaG = 0;
uint16_t dmaT = 0;
uint16_t dmaH = 0;
uint16_t dmaE = 0;
uint16_t dma = 0;

#define MAX_SAMPLES 1024
uint32_t dataPoints12[MAX_SAMPLES];


void ADC_init2() {
    // enable the ADC voltage regulator
    ADC1->CR &= ~ADC_CR_ADVREGEN_1;
    ADC2->CR &= ~ADC_CR_ADVREGEN_1;

    ADC1->CR |= ADC_CR_ADVREGEN_0;
    ADC2->CR |= ADC_CR_ADVREGEN_0;

    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1; // Prescaler ADC12 - PLL clock divided by 1
    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock

    // Set ADC clock
    // 00: (Asynchronous clock mode) PLL used
    // 01: HCLK/1 (Synchronous clock mode) AHB used
    // 10: HCLK/2
    // 11: HCLK/4
    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE_Msk;
    ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;

    // disable the ADC
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC2->CR &= ~ADC_CR_ADEN;

    // start ADC calibration cycle
    ADC1->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC1->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC2->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC2->CR & ADC_CR_ADCAL);

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;

    // wait till ADC start
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    // Select ADC Channels
    ADC1->SQR1 = (1u << ADC_SQR1_SQ1_Pos); // 1-st channel for ADC1
    ADC2->SQR1 = (2u << ADC_SQR1_SQ1_Pos); // 2-d  channel for ADC2

    // Set sampling time for regular group 1
    // 000: 1.5 ADC clock cycles
    // 001: 2.5 ADC clock cycles
    // 010: 4.5 ADC clock cycles
    // 011: 7.5 ADC clock cycles
    // 100: 19.5 ADC clock cycles
    // 101: 61.5 ADC clock cycles
    // 110: 181.5 ADC clock cycles
    // 111: 601.5 ADC clock cycles
    ADC1->SMPR1 |= (0b010u << ADC_SMPR1_SMP1_Pos);
    ADC2->SMPR1 |= (0b010u << ADC_SMPR1_SMP2_Pos);

    // Regular channel sequence length
    ADC1->SQR1 |= (0b0000u << ADC_SQR1_L_Pos); // One conversion in the regular sequence
    ADC2->SQR1 |= (0b0000u << ADC_SQR1_L_Pos);

    // set data resolution
    // 00: 12-bit
    // 01: 10-bit
    // 10: 8-bit
    // 11: 6-bit
    ADC1->CFGR |= 0b00u << ADC_CFGR_RES_Pos;
    ADC2->CFGR |= 0b00u << ADC_CFGR_RES_Pos;

    // Enable continuous conversion mode. Only on the masters
    ADC1->CFGR |= ADC_CFGR_CONT; // Master ADC1 + ADC2

    // dual mode. Set only when ADEN=0
    // 00000: Independent mode
    // 00110: Regular simultaneous mode only
    // 00111: Interleaved mode only
    ADC12_COMMON->CCR |= (0b00110u << ADC_CCR_DUAL_Pos);

    // DMA mode.  0 -> One Shot; 1 -> Circular
    ADC12_COMMON->CCR &= ~ADC_CCR_DMACFG;

    // DMA mode b10 - for 12-bit resolution
    ADC12_COMMON->CCR |= (0b10u << ADC_CCR_MDMA_Pos);
}

void DMA_init(void) {
    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Transfer complete interrupt enable
    DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;

    // Memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_MINC;

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << DMA_CCR_PSIZE_Pos);

    // Memory size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << DMA_CCR_MSIZE_Pos);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = MAX_SAMPLES;

    // Peripheral address register
    DMA1_Channel1->CPAR = (uint32_t) &ADC12_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR = (uint32_t) (&dataPoints12);

    // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
    DMA1->IFCR |= 0xFFu;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 CH1 interrupt
}

uint32_t elapsedTime = 0;

void ADC_takeSamples(void) {
    // Number of data to transfer
    DMA1_Channel1->CNDTR = MAX_SAMPLES;

    delay_us(10); // does not work without this random delay

    // Enable DMA
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    elapsedTime = DWT_Get();
    // ADC start conversion
    ADC1->CR |= ADC_CR_ADSTART;

    while ((DMA1_Channel1->CNDTR > 0));

    elapsedTime = DWT_Elapsed_Tick(elapsedTime) / DWT_IN_MICROSEC;

    // disable DMA channel
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    // ADC stop conversion
    ADC1->CR |= ADC_CR_ADSTP;

    // wait till ADC stop work
    while ((ADC1->CR & ADC_CR_ADSTART));
}

void ADC_init() {
    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB

    // Set mode b11 (analog input) for ADC pins
    GPIOA->MODER |= (0b11u << 0u); // PA0 for ADC1
    GPIOA->MODER |= (0b11u << 10u); // PA5 for ADC2 ch2

    DMA_init();
    ADC_init2();
}

void DMA1_Channel1_IRQHandler() {
    dma++;
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        dmaT++;
    }
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        dmaH++;
    }
    if (DMA1->ISR & DMA_ISR_TEIF1) {
        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        dmaE++;
    }
    if (DMA1->ISR & DMA_ISR_GIF1) {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        dmaG++;
    }
}
