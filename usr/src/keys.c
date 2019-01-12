#include <_main.h>
#include <keys.h>
#include <string.h>

#define DEBOUNCING_CNT 0
#define MAX_ENCODER    255 // max encoder value
#define MID_ENCODER    (MAX_ENCODER/2+1)
#define ENCODER_STEP   2   // counts per step
#define ENCODER_TIM    TIM8

uint8_t button1Count = 0;
uint8_t button2Count = 0;
uint8_t button3Count = 0;
uint16_t btns_state = 0;
static uint16_t debounceCnt = 0;
int16_t enc_step;

void KEYS_init() {
    ENCODER_TIM->CNT = MID_ENCODER;
}

int16_t ENC_Get() {
    int16_t result = 0;

    int16_t step = (int16_t) (ENCODER_TIM->CNT - MID_ENCODER);
    if (step >= ENCODER_STEP || step <= -ENCODER_STEP) {
        result = step / (int16_t) ENCODER_STEP;

        __disable_irq();
        ENCODER_TIM->CNT -= result * ENCODER_STEP;
        __enable_irq();
    }

    return result;
}

/**
 * Check buttons and run actions
 */
void KEYS_scan() {
    uint16_t new_state = (uint16_t) (0x1 & ~((BTN1_GPIO_Port->IDR & BTN1_Pin) >> 14)); // 14 pin number
    uint16_t action = (btns_state << 8) | new_state;
    btns_state = new_state;
    switch (action) {
        // BTN1 up
        case 0x0100:
            button1Count++;
            break;
        default:
            break;
    }

    // if encoder has step - do it
    enc_step = ENC_Get();
    if (enc_step == 0) return;
    char buf[64];
    sprintf(buf, "step: %hi\n", enc_step);
    DBG_Trace(buf);

    // choose type of encoder action
/*    int8_t action = button1Count % (int8_t) 3;
    if (action == 0) {
        ADC_step(step);
    } else if (action == 1) {
        GEN_step(step);
    } else {
        DAC_NextGeneratorSignal();
    } //*/
}
