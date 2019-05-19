#include <_main.h>
#include "adc.h"
#include "delay.h"

// https://stackoverflow.com/questions/46289034/stm32f303-adc-with-dma-only-works-a-few-times
// https://stackoverflow.com/questions/54024674/stm32f3-dual-adc-with-interleaved-mode

#define maxSamples 1024
uint8_t dataPoints12[maxSamples];
uint8_t dataPoints34[maxSamples];


void ADC_init() {
    // enable the ADC voltage regulator
    ADC1->CR &= ~(1u << 29u); // ADC_CR_ADVREGEN_0
    ADC2->CR &= ~(1u << 29u);
    ADC3->CR &= ~(1u << 29u);
    ADC4->CR &= ~(1u << 29u);

    ADC1->CR |= (1u << 28u);  // ADC_CR_ADVREGEN_1
    ADC2->CR |= (1u << 28u);
    ADC3->CR |= (1u << 28u);
    ADC4->CR |= (1u << 28u);

    RCC->CFGR2 |= (0b10000u << 4u); // Prescaler RCC_CFGR2_ADCPRE12_DIV1
    RCC->CFGR2 |= (0b10000u << 9u); // Prescaler RCC_CFGR2_ADCPRE34_DIV1
    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock
    RCC->AHBENR |= RCC_AHBENR_ADC34EN; // turn on ADC34 clock

    // Set ADC clock
    ADC12_COMMON->CCR |= (0b01u << 16u); // ADC_CCR_CKMODE_0
    ADC34_COMMON->CCR |= (0b01u << 16u); // ADC_CCR_CKMODE_0

    // disable the ADC
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC2->CR &= ~ADC_CR_ADEN;
    ADC3->CR &= ~ADC_CR_ADEN;
    ADC4->CR &= ~ADC_CR_ADEN;

    // start ADC calibration cycle
    ADC1->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC1->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC2->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC2->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC3->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC3->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC4->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC4->CR & ADC_CR_ADCAL);

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
    ADC3->CR |= ADC_CR_ADEN;
    ADC4->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & (1u << 0u))); // ADC_ISR_ADRDY
    while (!(ADC2->ISR & (1u << 0u)));
    while (!(ADC3->ISR & (1u << 0u)));
    while (!(ADC4->ISR & (1u << 0u)));

    // Select ADC Channels
    ADC1->SQR1 = (1u << 6u);
    ADC2->SQR1 = (1u << 6u);
    ADC3->SQR1 = (1u << 6u);
    ADC4->SQR1 = (3u << 6u);

    // Set sampling time for regular group 1
    ADC1->SMPR1 |= (0b000u << 3u); // 0b000 -> 1.5 clock cycles, shortest available sampling time
    ADC2->SMPR1 |= (0b000u << 3u); // ADC_SMPR1_SMP0
    ADC3->SMPR1 |= (0b000u << 3u);
    ADC4->SMPR1 |= (0b000u << 3u);

    // Regular sequence settings
    ADC1->SQR1 |= (0b0000u << 0u); // One conversion in the regular sequence
    ADC2->SQR1 |= (0b0000u << 0u);
    ADC3->SQR1 |= (0b0000u << 0u);
    ADC4->SQR1 |= (0b0000u << 0u);

    // Enable continuous conversion mode
    ADC1->CFGR |= (1u << 13u); // Master ADC1 + ADC2
    ADC3->CFGR |= (1u << 13u); // Master ADC3 + ADC4

    ADC12_COMMON->CCR |= (0b00110u << 0u);
    ADC34_COMMON->CCR |= (0b00110u << 0u);

    // DMA mode
    ADC12_COMMON->CCR |= (0u << 13u); // 0 -> One Shot; 1 -> Circular
    ADC34_COMMON->CCR |= (0u << 13u);

    // DMA mode for 12-bit resolution
    ADC12_COMMON->CCR |= (0b10u << 14u);
    ADC34_COMMON->CCR |= (0b10u << 14u);
}

void DMA_init(void) {

    // Enable clocks
    RCC->AHBENR |= (1u << 0u); // DMA1
    RCC->AHBENR |= (1u << 1u); // DMA2

    // Transfer complete interrupt enable
    DMA1_Channel1->CCR |= (1u << 1u);
    DMA2_Channel5->CCR |= (1u << 1u);

    // Memory increment mode
    DMA1_Channel1->CCR |= (1u << 7u);
    DMA2_Channel5->CCR |= (1u << 7u);

    // Peripheral size
    DMA1_Channel1->CCR |= (0b11u << 8u);
    DMA2_Channel5->CCR |= (0b11u << 8u);

    // Memory size
    DMA1_Channel1->CCR |= (0b11u << 10u);
    DMA2_Channel5->CCR |= (0b11u << 10u);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = (uint32_t)maxSamples;
    DMA2_Channel5->CNDTR = (uint32_t)maxSamples;

    // Peripheral address register
    DMA1_Channel1->CPAR |= (uint32_t) &ADC12_COMMON->CDR;
    DMA2_Channel5->CPAR |= (uint32_t) &ADC34_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR |= (uint32_t)(&dataPoints12);
    DMA2_Channel5->CMAR |= (uint32_t)(&dataPoints34);

    // Reset flags
    DMA1->IFCR |= 0xFFu;
    DMA2->IFCR |= 0xFFu;
}

void takeSamples(void) {

    // Reset flags
    DMA1->IFCR |= (0b1111111111111111111111111111111u << 0u);
    DMA2->IFCR |= (0b1111111111111111111111111111111u << 0u);

    // Number of data to transfer
    DMA1_Channel1->CNDTR = (uint32_t)(maxSamples);
    DMA2_Channel5->CNDTR = (uint32_t)(maxSamples);

    delay_us(10); // does not work without this random delay

//    elapsedTime = micros();
    // Enable DMA
    DMA1_Channel1->CCR |= (1u << 0u);
    DMA2_Channel5->CCR |= (1u << 0u);

    while ((DMA1_Channel1->CNDTR > 0) || (DMA2_Channel5->CNDTR > 0)) {
    }

//    elapsedTime = micros() - elapsedTime;

// Reset flags
    DMA1->IFCR |= (0b1111111111111111111111111111111u << 0u);
    DMA2->IFCR |= (0b1111111111111111111111111111111u << 0u);

    DMA1_Channel1->CCR &= ~(1u << 0u);
    DMA2_Channel5->CCR &= ~(1u << 0u);

// ADC stop conversion
    ADC1->CR |= (1u << 4u);
    ADC3->CR |= (1u << 4u);

    while ((ADC1->CR & (1u << 2u)) || (ADC3->CR & (1u << 2u)));

    ADC12_COMMON->CCR &= ~(0b10u << 14u);
    ADC34_COMMON->CCR &= ~(0b10u << 14u);

    ADC12_COMMON->CCR |= (0b10u << 14u);
    ADC34_COMMON->CCR |= (0b10u << 14u);

// ADC start conversion
    ADC1->CR |= (1u << 2u);
    ADC3->CR |= (1u << 2u);
}

//void loop() {
//    takeSamples();
//    Serial.print("Elapsed time: ");
//    Serial.println(elapsedTime);
//}

void ADC_setup() {
//    Serial.begin(57600);

    // Enable clocks
    RCC->AHBENR |= (1u << 17u); // GPIOA
    RCC->AHBENR |= (1u << 18u); // GPIOB

    // Set ADC pins to analog input
    GPIOA->MODER |= (0b11u << 0u); // PA0 for ADC1
    GPIOA->MODER |= (0b11u << 8u); // PA4 for ADC2
    GPIOB->MODER |= (0b11u << 2u); // PB1 for ADC3
    GPIOB->MODER |= (0b11u << 24u); // PB1 for ADC4

//    initClock();
    DMA_init();
    ADC_init();

    // Start conversion
    ADC1->CR |= (1u << 2u);
    ADC3->CR |= (1u << 2u);
}
