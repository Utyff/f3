#ifndef F3_ADC_H
#define F3_ADC_H

void ADC_Init();

void ADC_start() __attribute__((section (".ccmram")));


extern uint8_t dualMode;
extern uint8_t adcResolution;
extern uint8_t rccAdcDivider;
extern uint8_t adcDelay;
extern uint8_t sampleTime;

#endif //F3_ADC_H
