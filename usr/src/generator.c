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
    int Frequency;    // Hz
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

    tim1Prescaler = GEN_Parameters[currentGenParam].TIM_Prescaler;
    tim1Period = GEN_Parameters[currentGenParam].TIM_Period * currentGenScale;
    tim1Pulse = tim1Period * 30 / 100;
    GEN_setParams();

    sprintf(msg, "After step. param: %u, scale: %u, presc: %u, period: %u\n", currentGenParam, currentGenScale, tim1Prescaler, tim1Period);
    DBG_Trace(msg);
}

void GEN_setFreq() {
    GEN_setParams();
}

/**
 *  made from MX_TIM1_Init()
 *
 */
void GEN_setParams() {
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = tim1Prescaler;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = tim1Period;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
        Error_Handler();

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = tim1Pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
