#ifndef F3_GENERATOR_H
#define F3_GENERATOR_H

extern uint32_t tim1Prescaler;
extern uint32_t tim1Period;
extern uint32_t tim1Pulse;
extern uint32_t tim1Freq;

extern uint32_t tim1Freq;

void GEN_Init();

void GEN_step(int16_t step);

void GEN_setFreq();

void GEN_setParams();

#endif //F3_GENERATOR_H
