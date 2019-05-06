#include <stm32f3xx.h>
#include "main.h"
#include <_main.h>
#include <delay.h>

#define TIM_CHANNEL_1   (0x0000U)
#define TIM_CHANNEL_2   (0x0004U)
#define TIM_CHANNEL_3   (0x0008U)
#define TIM_CHANNEL_4   (0x000CU)
#define TIM_CHANNEL_5   (0x0010U)
#define TIM_CHANNEL_6   (0x0014U)

void SystemClock_Config(void);

// init TIM1 as pwm
void TIM1_init() {
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
    TIM1->PSC = 14399; // 144MHz / 14400 = 10 KHz
    // Set the Autoreload value
    TIM1->ARR = 4999;  // 10 KHz / 5000 = 2Hz
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // interrupt on update

    // Configure the Channel 1 in PWM mode
    /* Disable the Channel 1: Reset the CC1E Bit */
    TIM1->CCER &= ~TIM_CCER_CC1E;
    /* Select the Output Compare Mode */
    TIM1->CCMR1 |= ((uint32_t)TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    /* Set the Capture Compare Register value */
    TIM1->CCR1 = 4000;
    // Set the Preload enable bit for channel1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    // Configure the Output Fast mode
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1FE;
    // Enable the Capture compare channel
    TIM1->CCER |= TIM_CCER_CC1E;
    // Enable the main output
    TIM1->BDTR|=TIM_BDTR_MOE;
    /* Enable the Peripheral */
    TIM1->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    // Configure the system clock
    SystemClock_Config();

    mainInitialize();

    // Initialize LED GPIO
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    // OUT mode
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER13)) | (GPIO_MODER_MODER13_0);
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER15)) | (GPIO_MODER_MODER15_0);

    TIM1_init();

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

    // Infinite loop
    while (1) {
        mainCycle();
    }
#pragma clang diagnostic pop
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    // External oscillator (HSE) = 24MHz
    // SYS_CLK = 72MHz
    // APB1 = 36MHz
    RCC->CR |= RCC_CR_HSEON;           // Enable HSE
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    RCC->CFGR |= RCC_CFGR_PLLMUL3;     // PLL MUL = x3, SYS_CLK=72MHz (x9 for 8MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 Prescaler = 2
    RCC->CFGR |= RCC_CFGR_PLLSRC;      // PLL source = HSE

    FLASH->ACR |= FLASH_ACR_LATENCY_1; // Two wait states

    RCC->CR |= RCC_CR_PLLON;           // Enable and wait PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_SW_PLL;      // Select PLL as system clock

//    RCC_HSICmd(DISABLE);//Disable HSI
}

uint32_t tim1count = 0;

void TIM1_UP_TIM16_IRQHandler() {
    tim1count++;
    TIM1->SR = (uint16_t) ~TIM_SR_UIF;
    GPIOC->ODR ^= GPIO_ODR_15;
}

uint32_t tim2count = 0;

void TIM1_CC_IRQHandler() {
    tim2count++;
    TIM1->SR = (uint16_t) ~TIM_SR_CC1IF;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void) {
//#pragma clang diagnostic push
//#pragma clang diagnostic ignored "-Wmissing-noreturn"
//    while (1) {
//    }
//#pragma clang diagnostic pop
//}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
