#ifndef F3_ADC_H
#define F3_ADC_H

struct ADC_param {
    uint8_t rccDivider;
    uint8_t interleaveDelay;
    uint8_t sampleTime;
};
typedef struct ADC_param ADC_PARAM;

extern const ADC_PARAM ADC_Parameters[];
extern int currentADCParam;

void ADC_Init();

void ADC_start() __attribute__((section (".ccmram")));

void ADC_step(int16_t step);

#endif //F3_ADC_H
