#include <_main.h>
#include "adc.h"
#include "delay.h"

// https://stackoverflow.com/questions/46289034/stm32f303-adc-with-dma-only-works-a-few-times
// https://stackoverflow.com/questions/54024674/stm32f3-dual-adc-with-interleaved-mode

uint16_t dmaG = 0;
uint16_t dmaT = 0;
uint16_t dmaH = 0;
uint16_t dmaE = 0;

#define MAX_SAMPLES 1024
uint32_t dataPoints12[MAX_SAMPLES];
uint32_t dataPoints34[MAX_SAMPLES];


void ADC_set() {
    // enable the ADC voltage regulator
    ADC1->CR &= ~(1u << 29u); // ADC_CR_ADVREGEN_0
    ADC2->CR &= ~(1u << 29u);
//    ADC3->CR &= ~(1u << 29u);
//    ADC4->CR &= ~(1u << 29u);

    ADC1->CR |= (1u << 28u);  // ADC_CR_ADVREGEN_1
    ADC2->CR |= (1u << 28u);
//    ADC3->CR |= (1u << 28u);
//    ADC4->CR |= (1u << 28u);

    RCC->CFGR2 |= (0b10000u << 4u); // Prescaler RCC_CFGR2_ADCPRE12_DIV1
//    RCC->CFGR2 |= (0b10000u << 9u); // Prescaler RCC_CFGR2_ADCPRE34_DIV1
    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock
//    RCC->AHBENR |= RCC_AHBENR_ADC34EN; // turn on ADC34 clock

    // Set ADC clock
    ADC12_COMMON->CCR |= (0b01u << 16u); // ADC_CCR_CKMODE_0
//    ADC34_COMMON->CCR |= (0b01u << 16u); // ADC_CCR_CKMODE_0

    // disable the ADC
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC2->CR &= ~ADC_CR_ADEN;
//    ADC3->CR &= ~ADC_CR_ADEN;
//    ADC4->CR &= ~ADC_CR_ADEN;

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

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
//    ADC3->CR |= ADC_CR_ADEN;
//    ADC4->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & (1u << 0u))); // ADC_ISR_ADRDY
    while (!(ADC2->ISR & (1u << 0u)));
//    while (!(ADC3->ISR & (1u << 0u)));
//    while (!(ADC4->ISR & (1u << 0u)));

    // Select ADC Channels
    ADC1->SQR1 = (1u << 6u);
    ADC2->SQR1 = (1u << 6u);
//    ADC3->SQR1 = (1u << 6u);
//    ADC4->SQR1 = (3u << 6u);

    // Set sampling time for regular group 1
    ADC1->SMPR1 |= (0b000u << 3u); // 0b000 -> 1.5 clock cycles, shortest available sampling time
    ADC2->SMPR1 |= (0b000u << 3u); // ADC_SMPR1_SMP0
//    ADC3->SMPR1 |= (0b000u << 3u);
//    ADC4->SMPR1 |= (0b000u << 3u);

    // Regular sequence settings
    ADC1->SQR1 |= (0b0000u << 0u); // One conversion in the regular sequence
    ADC2->SQR1 |= (0b0000u << 0u);
//    ADC3->SQR1 |= (0b0000u << 0u);
//    ADC4->SQR1 |= (0b0000u << 0u);

    // set data resolution
    // 00: 12-bit
    // 01: 10-bit
    // 10: 8-bit
    // 11: 6-bit
    ADC1->CFGR |= 0b00u << ADC_CFGR_RES_Pos;
    ADC2->CFGR |= 0b00u << ADC_CFGR_RES_Pos;
//    ADC3->CFGR |= 0b00u << ADC_CFGR_RES_Pos;
//    ADC4->CFGR |= 0b00u << ADC_CFGR_RES_Pos;

    // Enable continuous conversion mode. Only on the masters
    ADC1->CFGR |= (1u << 13u); // Master ADC1 + ADC2
//    ADC3->CFGR |= (1u << 13u); // Master ADC3 + ADC4

    // dual mode - Regular simultaneous mode only
    ADC12_COMMON->CCR |= (0b00110u << 0u);
//    ADC34_COMMON->CCR |= (0b00110u << 0u);

    // DMA mode.  0 -> One Shot; 1 -> Circular
    ADC12_COMMON->CCR |= (0u << 13u);
//    ADC34_COMMON->CCR |= (0u << 13u);

    // DMA mode for 12-bit resolution
    ADC12_COMMON->CCR |= (0b10u << 14u);
//    ADC34_COMMON->CCR |= (0b10u << 14u);
}

void DMA_init(void) {

    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // (1u << 0u); // DMA1
//    RCC->AHBENR |= (1u << 1u); // DMA2

    // Transfer complete interrupt enable
    DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;
//    DMA2_Channel5->CCR |= (1u << 1u);

    // Memory increment mode
    DMA1_Channel1->CCR |= (1u << 7u);
//    DMA2_Channel5->CCR |= (1u << 7u);

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << 8u);
//    DMA2_Channel5->CCR |= (0b11u << 8u);

    // Memory size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b10u << 10u);
//    DMA2_Channel5->CCR |= (0b11u << 10u);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = MAX_SAMPLES;
//    DMA2_Channel5->CNDTR = MAX_SAMPLES;

    // Peripheral address register
    DMA1_Channel1->CPAR = (uint32_t) &ADC12_COMMON->CDR;
//    DMA2_Channel5->CPAR = (uint32_t) &ADC34_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR = (uint32_t)(&dataPoints12);
//    DMA2_Channel5->CMAR = (uint32_t)(&dataPoints34);

    // Reset flags
    DMA1->IFCR |= 0xFFu;
//    DMA2->IFCR |= 0xFFu;
}

void ADC_takeSamples(void) {

    // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
    DMA1->IFCR |= (0b1111111111111111111111111111111u << 0u);
//    DMA2->IFCR |= (0b1111111111111111111111111111111u << 0u);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = MAX_SAMPLES;
//    DMA2_Channel5->CNDTR = MAX_SAMPLES;

    delay_us(10); // does not work without this random delay

//    elapsedTime = micros();
    // Enable DMA
    DMA1_Channel1->CCR |= (1u << 0u);
//    DMA2_Channel5->CCR |= (1u << 0u);

    while ((DMA1_Channel1->CNDTR > 0));
//      || (DMA2_Channel5->CNDTR > 0)) {
//    }

//    elapsedTime = micros() - elapsedTime;

    // Reset flags
    DMA1->IFCR |= (0b1111111111111111111111111111111u << 0u);
//    DMA2->IFCR |= (0b1111111111111111111111111111111u << 0u);

    // disable DMA channel
    DMA1_Channel1->CCR &= ~(1u << 0u);
//    DMA2_Channel5->CCR &= ~(1u << 0u);

    // ADC stop conversion
    ADC1->CR |= (1u << 4u);
//    ADC3->CR |= (1u << 4u);

    while ((ADC1->CR & (1u << 2u)));
//    || (ADC3->CR & (1u << 2u)));

    ADC12_COMMON->CCR &= ~(0b10u << 14u);
//    ADC34_COMMON->CCR &= ~(0b10u << 14u);

    ADC12_COMMON->CCR |= (0b10u << 14u);
//    ADC34_COMMON->CCR |= (0b10u << 14u);

    // ADC start conversion
    ADC1->CR |= (1u << 2u);
//    ADC3->CR |= (1u << 2u);
}

//void loop() {
//    ADC_takeSamples();
//    Serial.print("Elapsed time: ");
//    Serial.println(elapsedTime);
//}

void ADC_init() {
//    Serial.begin(57600);

    // Enable clocks
    RCC->AHBENR |= (1u << 17u); // GPIOA
    RCC->AHBENR |= (1u << 18u); // GPIOB

    // Set ADC pins to analog input
    GPIOA->MODER |= (0b11u << 0u); // PA0 for ADC1
//    GPIOA->MODER |= (0b11u << 8u); // PA4 for ADC2
//    GPIOB->MODER |= (0b11u << 2u); // PB1 for ADC3
//    GPIOB->MODER |= (0b11u << 24u); // PB1 for ADC4

//    initClock();
    DMA_init();
    ADC_set();

    // Start conversion
    ADC1->CR |= (1u << 2u);
//    ADC3->CR |= (1u << 2u);
}

void DMA1_Channel1_IRQHandler() {
    if( DMA1->ISR | DMA_ISR_TCIF1 ) {
        // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        dmaT++;
    }
    if( DMA1->ISR | DMA_ISR_HTIF1 ) {
        // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        dmaH++;
    }
    if( DMA1->ISR | DMA_ISR_TEIF1 ) {
        // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        dmaE++;
    }
    if( DMA1->ISR | DMA_ISR_GIF1 ) {
        // Reset flags  DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        dmaG++;
    }
}