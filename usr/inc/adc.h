#ifndef F7_FMC_ADC_H
#define F7_FMC_ADC_H

//extern uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
extern uint32_t ADCElapsedTick;       // the last time buffer fill

void ADC_setParams();
void ADC_step(int16_t step);

#endif //F7_FMC_ADC_H
