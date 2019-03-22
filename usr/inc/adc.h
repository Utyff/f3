#ifndef F7_FMC_ADC_H
#define F7_FMC_ADC_H

#include <stm32f303xc.h>

/**
  * ADCEx_sampling_times ADC Extended Sampling Times
  *  from stm32f3xx_hal_adc_ex.h
  */
#define ADC_SAMPLETIME_1CYCLE_5       (0x00000000U)                              /*!< Sampling time 1.5 ADC clock cycle */
#define ADC_SAMPLETIME_2CYCLES_5      ((uint32_t)ADC_SMPR2_SMP10_0)                       /*!< Sampling time 2.5 ADC clock cycles */
#define ADC_SAMPLETIME_4CYCLES_5      ((uint32_t)ADC_SMPR2_SMP10_1)                       /*!< Sampling time 4.5 ADC clock cycles */
#define ADC_SAMPLETIME_7CYCLES_5      ((uint32_t)(ADC_SMPR2_SMP10_1 | ADC_SMPR2_SMP10_0)) /*!< Sampling time 7.5 ADC clock cycles */
#define ADC_SAMPLETIME_19CYCLES_5     ((uint32_t)ADC_SMPR2_SMP10_2)                       /*!< Sampling time 19.5 ADC clock cycles */
#define ADC_SAMPLETIME_61CYCLES_5     ((uint32_t)(ADC_SMPR2_SMP10_2 | ADC_SMPR2_SMP10_0)) /*!< Sampling time 61.5 ADC clock cycles */
#define ADC_SAMPLETIME_181CYCLES_5    ((uint32_t)(ADC_SMPR2_SMP10_2 | ADC_SMPR2_SMP10_1)) /*!< Sampling time 181.5 ADC clock cycles */
#define ADC_SAMPLETIME_601CYCLES_5    ((uint32_t)ADC_SMPR2_SMP10)                         /*!< Sampling time 601.5 ADC clock cycles */

//extern uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
extern uint32_t ADCElapsedTick;       // the last time buffer fill

void ADC_setParams();
void ADC_step(int16_t step);

#endif //F7_FMC_ADC_H
