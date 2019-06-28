#ifndef KEYS_H
#define KEYS_H

#include "_main.h"

// buttons bits
#define BUTTON1 0x01
#define BUTTON2 0x02
#define BUTTON3 0x04
#define BUTTON4 0x08

#define KEY_MODE_ADC 0
#define KEY_MODE_GEN 1
#define KEY_MODE_MAX 1

extern uint8_t button1Count;
extern uint8_t button2Count;
extern uint8_t button3Count;
extern uint16_t btns_state;
extern uint16_t new_state;
extern int16_t enc_step;
extern int16_t enc_count;
extern int16_t keyMode;

void KEYS_Init();
void KEYS_scan();
void ENC_init();
int16_t ENC_Get();

#endif //KEYS_H
