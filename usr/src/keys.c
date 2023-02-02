#include <_main.h>
#include <dwt.h>
#include <keys.h>
#include <generator.h>
#include <stdio.h>

// 50 millisecond
#define DEBOUNCING_TIMEOUT (50 * DWT_IN_MICROSEC * 1000)
#define MAX_ENCODER    255 // max encoder value
#define MID_ENCODER    (MAX_ENCODER/2+1)
#define ENCODER_STEP   2   // counts per step
#define ENCODER_TIM    TIM8

uint8_t button1Count = 0;
uint8_t button2Count = 0;
uint8_t button3Count = 0;
uint16_t btns_state = 0;
static uint32_t debounceStart;
int16_t enc_step;
int16_t keyMode = KEY_MODE_ADC;

void KEYS_Init() {
    debounceStart = DWT_Get();
    ENC_init();

    // Init buttons enable GPIOC
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    // PC13,14,15 IN mode - 00
    GPIOC->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
    // Pull up
    GPIOC->PUPDR = GPIO_PUPDR_PUPDR13_0 | GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR15_0;
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
uint16_t new_state;
int16_t enc_count = 1000;

void KEYS_scan() {
    new_state = (uint16_t) ((~BTN1_GPIO_Port->IDR >> 13u) & 7u); // get buttons 1-3
    // debounce
    if (new_state != btns_state) {
        if ((DWT_Get() - debounceStart) < DEBOUNCING_TIMEOUT) {
            return;
        }
        debounceStart = DWT_Get();
    }

    uint16_t action = (btns_state << 8u) | new_state; // action code = last btn state + new btn state
    btns_state = new_state;
    switch (action) {
        case 0x0100: // BTN1 up
            if (++keyMode > KEY_MODE_MAX) keyMode = 0;
            button1Count++;
            break;

        case 0x0200: // BTN2 up
            button2Count++;
            break;

        case 0x0400: // BTN3 up
            button3Count++;
            break;

        default:
            break;
    }

    // if encoder has step - do it
    enc_step = ENC_Get();
    enc_count += enc_step;
    if (enc_step == 0) return;
    char buf[64];
    sprintf(buf, "step: %hi\n", enc_step);
    DBG_Trace(buf);

    // choose encoder action
    if (keyMode == 1) {
        GEN_step(enc_step);
    }
}

// Init TIM8 as encoder
// TIM8 GPIO Configuration
// PA15 AF2  -> TIM8_CH1
// PB8  AF10 -> TIM8_CH2
void ENC_init() {
    // GPIO on
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // AF2 for TIM8CH1 signals for PA15
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH0)) | (2u << (7 * 4u));
    // Select AF mode (10) on PA15
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER15)) | (GPIO_MODER_MODER15_1);
    // Pull up PA15
    GPIOA->PUPDR = GPIO_PUPDR_PUPDR15_0;
    // AF10 for TIM8CH2 signals for PB8
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(GPIO_AFRH_AFRH0)) | (10u << (0 * 4u));
    // Select AF mode (10) on PB8
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1);
    // Pull up PB8
    GPIOB->PUPDR = GPIO_PUPDR_PUPDR8_0;

    // TIM8 on
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    // set PLL_CLK*2 as source TIM8
    RCC->CFGR3 |= RCC_CFGR3_TIM8SW_PLL;
    // Reset the SMS bits
    TIM8->SMCR &= ~TIM_SMCR_SMS;
    // Set the Prescaler value
    TIM8->PSC = 0;
    // Set the Autoreload value
    TIM8->ARR = 255;
    // Set the Repetition Counter value
    TIM8->RCR = 0;
    // Generate an update event to reload the Prescaler
    // and the repetition counter(only for TIM1 and TIM8) value immediatly
    TIM8->EGR = TIM_EGR_UG;

    // set TIM_ENCODERMODE_TI12
    TIM8->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;

    // Select the Capture Compare 1 and the Capture Compare 2 as input - TIM_ICSELECTION_DIRECTTI
    // TIM Input 1U, 2U, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
    MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_CC1S | TIM_CCMR1_CC2S, TIM_CCMR1_CC1S_0 | (TIM_CCMR1_CC1S_0 << 8U));
    // Set the Capture Compare 1 and the Capture Compare 2 prescalers and filters
    TIM8->CCMR1 &= ~(TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
    MODIFY_REG(TIM8->CCMR1, (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F), (6u << 4U) | (6u << 12U));
    // Polarity for TI1 and TI2 source - TIM_INPUTCHANNELPOLARITY_RISING
    TIM8->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);

    // enable chanel 1
    TIM8->CCER |= TIM_CCER_CC1E;
    // enable TIM8
    TIM8->CR1 |= TIM_CR1_CEN;

    // set counter at the middle
    ENCODER_TIM->CNT = MID_ENCODER;
}
