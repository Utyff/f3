#ifndef F3_KEYS_H
#define F3_KEYS_H

#include "_main.h"

// buttons bits
#define BUTTON1 0x01
#define BUTTON2 0x02
#define BUTTON3 0x04
#define BUTTON4 0x08

extern uint8_t button1Count;
extern uint8_t button2Count;
extern uint8_t button3Count;
extern uint16_t btns_state;
extern uint16_t new_state;
extern int16_t enc_step;
extern int16_t enc_count;

void KEYS_init();
void KEYS_scan();
void ENC_init();
int16_t ENC_Get();

#endif //F3_KEYS_H
