#ifndef GENERATOR_H
#define GENERATOR_H

extern uint32_t tim1Prescaler;
extern uint32_t tim1Period;
extern uint32_t tim1Pulse;
extern uint32_t tim1Freq;

void GEN_Init();

void GEN_step(int16_t step);

void GEN_setFreq();

void GEN_setParams();

#endif //GENERATOR_H
