#include <_main.h>
#include "generator.h"

/* F3
 * TIM1 Configuration
 * CLK  72 mHz
 * TIM1 CLK 144
 * PRE           72 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */
/* F7
 * TIM1 Configuration
 * CLK  216 mHz
 * PRE           108 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */
/* H7
 * TIM1 Configuration
 * CLK  - 400 mHz
 * AHB2 - 200 mHz
 * PRE           100 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */

struct GEN_param {
    uint32_t TIM_Prescaler;
    uint32_t TIM_Period;
    uint32_t Frequency;    // Hz
};
typedef struct GEN_param GEN_PARAM;

#define GEN_Parameters_Size 6
const GEN_PARAM GEN_Parameters[GEN_Parameters_Size] = {
        {71, 19,  100000},
        {71, 24,  80000},
        {49, 47,  60000},
        {71, 49,  40000},
        {71, 99,  20000},
        {71, 199, 10000}
};

int currentGenParam = 4;
int currentGenScale = 1;

// start params - 20000 Hz
uint32_t tim1Prescaler = 71;
uint32_t tim1Period = 99;
uint32_t tim1Pulse = 30;
uint32_t tim1Freq = 20000;

void GEN_step(int16_t step) {
    char msg[200];

    if (step == 0) return;

    if (step > 0) currentGenParam--;
    else currentGenParam++;

    // up Freq
    if (currentGenParam < 0) {
        currentGenParam += 5;
        currentGenScale /= 10;
        if (currentGenScale < 1) {
            currentGenParam = 0;
            currentGenScale = 1;
        }
    }

    // down Freq
    if (currentGenParam > 4) {
        currentGenParam -= 5;
        currentGenScale *= 10;
        if (currentGenScale > 100) {
            currentGenParam = 4;
            currentGenScale = 100;
        }
    }

    tim1Freq = GEN_Parameters[currentGenParam].Frequency * currentGenScale;
    tim1Prescaler = GEN_Parameters[currentGenParam].TIM_Prescaler;
    tim1Period = GEN_Parameters[currentGenParam].TIM_Period * currentGenScale;
    tim1Pulse = tim1Period * 30 / 100;
    GEN_setParams();

//    sprintf(msg, "Timer param: %u, scale: %u, presc: %u, period: %u, freq: %u\n", currentGenParam, currentGenScale, tim1Prescaler, tim1Period, tim1Freq);
    DBG_Trace(msg);
}

void GEN_setParams() {
    // Set the Prescaler value
    TIM1->PSC = tim1Prescaler; // 144MHz /144 = 1mHz      / 14400 = 10 KHz
    // Set the Autoreload value
    TIM1->ARR = tim1Period;  // 10 KHz / 5000 = 2Hz
    // Set the Capture Compare Register value
    TIM1->CCR1 = tim1Pulse;
}

// init TIM1 as pwm
void GEN_Init() {
    // set PA8 for TIM1 CH1
    // Very high speed PA8
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
    // AF6 for TIM1CH1 signals for PA8
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH0)) | (6u << (0 * 4u));
    // Select AF mode (10) on PA8
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1);

    // Initialize TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->CFGR3 |= RCC_CFGR3_TIM1SW_PLL; // set PLL_CLK*2 as source TIM1
    // Set the Prescaler value
    TIM1->PSC = 143; //tim1Prescaler; // 144MHz /144 = 1mHz      / 14400 = 10 KHz
    // Set the Autoreload value
    TIM1->ARR = 499; //tim1Period;  // 10 KHz / 5000 = 2Hz
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // interrupt on update

    // Configure the Channel 1 in PWM mode
    // Disable the Channel 1: Reset the CC1E Bit
    TIM1->CCER &= ~TIM_CCER_CC1E;
    // Select the Output Compare Mode
    TIM1->CCMR1 |= ((uint32_t) TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    // Set the Capture Compare Register value
    TIM1->CCR1 = 400; //tim1Pulse;
    // Set the Preload enable for channel1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    // Configure the Output Fast mode
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1FE;
    // Enable the Capture compare channel
    TIM1->CCER |= TIM_CCER_CC1E;
    // Enable the main output
    TIM1->BDTR |= TIM_BDTR_MOE;
    // Enable the Peripheral
    TIM1->CR1 |= TIM_CR1_CEN;
}
