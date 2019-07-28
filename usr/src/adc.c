#include <_main.h>
#include <adc.h>
#include <delay.h>
#include <DataBuffer.h>


/**
 * ADC1 channel 1 - PA0
 * ADC2 channel 2 - PA5
 */

#define ADC_SINGLE_MODE 0
#define ADC_DUAL_MODE 1
uint8_t dualMode = ADC_SINGLE_MODE;

uint8_t firstInit = 0;

#define ADC_6BITS  0b11u
#define ADC_8BITS  0b10u
#define ADC_10BITS 0b01u
#define ADC_12BITS 0b00u
uint8_t adcResolution = ADC_8BITS;

// 0b10000: PLL clock divided by 1  // RCC_CFGR2_ADCPRE12_DIV1
// 0b10001: PLL clock divided by 2
// 0b10010: PLL clock divided by 4
// 0b10011: PLL clock divided by 6
// 0b10100: PLL clock divided by 8
// 0b10101: PLL clock divided by 10
// 0b10110: PLL clock divided by 12
// 0b10111: PLL clock divided by 16
// 0b11000: PLL clock divided by 32
// 0b11001: PLL clock divided by 64
// 0b11010: PLL clock divided by 128
// 0b11011: PLL clock divided by 256
uint8_t rccAdcDivider = 0b10000;

// Delay for interleaved mode. Set only when ADEN=0
// (SAMPLE_TIME + CONV. TIME) /2
// (4.5 + 12.5) /2 = 8 tics
// 0b0000 - 1
// 0b0001 - 2
// 0b0010 - 3
// 0b0011 - 4   MAX: 1011 - 12
uint8_t adcDelay = 0b1011;

//000: 1.5 ADC clock cycles
//001: 2.5 ADC clock cycles
//010: 4.5 ADC clock cycles
//011: 7.5 ADC clock cycles
//100: 19.5 ADC clock cycles
//101: 61.5 ADC clock cycles
//110: 181.5 ADC clock cycles
//111: 601.5 ADC clock cycles
uint8_t sampleTime = 0b001;


void ADC_Disable();

void DMA_Init();


void ADC_Init() {
    DMA_Init();

        // Enable clocks GPIOA
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

        // Set mode b11 (analog input) for ADC pins
        GPIOA->MODER |= (0b11u << 0u);  // PA0 for ADC1 ch1
        GPIOA->MODER |= (0b11u << 10u); // PA5 for ADC2 ch2

        // enable the ADC voltage regulator
        ADC1->CR &= ~ADC_CR_ADVREGEN_1;
        ADC2->CR &= ~ADC_CR_ADVREGEN_1;

        ADC1->CR |= ADC_CR_ADVREGEN_0;
        ADC2->CR |= ADC_CR_ADVREGEN_0;
        delay_us(10); // voltage regulator startup time

        RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock

        // Set Prescaler ADC for Asynchronous clock mode
        RCC->CFGR2 = MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12_Msk, rccAdcDivider << RCC_CFGR2_ADCPRE12_Pos);

        // Set ADC clock
        // 00: (Asynchronous clock mode) PLL
        // 01: HCLK/1 (Synchronous clock mode) AHB
        // 10: HCLK/2
        // 11: HCLK/4
        ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE_Msk;
        ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;

        // start ADC calibration cycle
        ADC1->CR |= ADC_CR_ADCAL;
        // wait for calibration to complete
        while (ADC1->CR & ADC_CR_ADCAL);

        // start ADC calibration cycle
        ADC2->CR |= ADC_CR_ADCAL;
        // wait for calibration to complete
        while (ADC2->CR & ADC_CR_ADCAL);

    // Set Prescaler ADC for Asynchronous clock mode
    RCC->CFGR2 = MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12_Msk, rccAdcDivider << RCC_CFGR2_ADCPRE12_Pos);

    // Delay for interleaved mode. Set only when ADEN=0
    ADC12_COMMON->CCR |= (adcDelay << ADC_CCR_DELAY_Pos);

    // dual mode. Set only when ADEN=0
    // 00000: Independent mode
    // 00110: Regular simultaneous mode only
    // 00111: Interleaved mode only
    ADC12_COMMON->CCR |= (0b00110u << ADC_CCR_DUAL_Pos);

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;

    // wait till ADC start
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    // Select ADC Channels
    ADC1->SQR1 = (1u << ADC_SQR1_SQ1_Pos); // 1-st channel for ADC1
    ADC2->SQR1 = (2u << ADC_SQR1_SQ1_Pos); // 2-d  channel for ADC2

    // Set sampling time for channels
    ADC1->SMPR1 |= (sampleTime << ADC_SMPR1_SMP1_Pos);
    ADC2->SMPR1 |= (sampleTime << ADC_SMPR1_SMP2_Pos);

    // Regular channel sequence length - 1
    ADC1->SQR1 |= (0b0000u << ADC_SQR1_L_Pos);
    ADC2->SQR1 |= (0b0000u << ADC_SQR1_L_Pos);

    // set data resolution
    ADC1->CFGR |= adcResolution << ADC_CFGR_RES_Pos;
    ADC2->CFGR |= adcResolution << ADC_CFGR_RES_Pos;

    // Enable continuous conversion mode. Only on the masters
    ADC1->CFGR |= ADC_CFGR_CONT; // Master ADC1 + ADC2

    // DMA mode.  0 -> One Shot; 1 -> Circular
    // 0 - stop when DMA_CCR_TCIE
    ADC12_COMMON->CCR &= ~ADC_CCR_DMACFG;

    // COMMON DMA mode b11 - for 8-bit resolution
    ADC12_COMMON->CCR |= (0b11u << ADC_CCR_MDMA_Pos);
}

void DMA_Init() {
    // Enable clocks DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Transfer complete interrupt enable
    DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;

    // Memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_MINC;

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b01u << DMA_CCR_PSIZE_Pos);

    // Memory size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b01u << DMA_CCR_MSIZE_Pos);

    // Number of data to transfer. 2 samples in 1 transfer.
    DMA1_Channel1->CNDTR = BUF_SIZE / 2;

    // Peripheral address register
    DMA1_Channel1->CPAR = (uint32_t) &ADC12_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR = (uint32_t) (&samplesBuffer);

    // Reset interrupt flags
    DMA1->IFCR |= DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 CH1 interrupt
}

uint32_t elapsedTime = 0;

void ADC_StartSamples(void) {
    // Number of data to transfer. 2 samples in 1 transfer.
    DMA1_Channel1->CNDTR = BUF_SIZE / 2;

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

uint16_t dmaG = 0;
uint16_t dmaT = 0;
uint16_t dmaH = 0;
uint16_t dmaE = 0;
uint16_t dma = 0;

void DMA1_Channel1_IRQHandler() {
    dma++;
    if (DMA1->ISR & DMA_ISR_TCIF1) { // transfer complete
        samplesReady = 1;
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        dmaT++;
    }
    if (DMA1->ISR & DMA_ISR_HTIF1) { // half transfer
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
