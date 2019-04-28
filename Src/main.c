#include <stm32f3xx.h>
#include "main.h"
#include <_main.h>
#include <delay.h>


void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    // Configure the system clock
    SystemClock_Config();
    DWT_Init();

    // Initialize LED GPIO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER13)) | (GPIO_MODER_MODER13_0);
    GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER15)) | (GPIO_MODER_MODER15_0);

    // Initialize TIM1
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    //RCC->CFGR3 &= ~RCC_CFGR3_TIM1SW_PLL; // set as default

    // Set the Prescaler value
    TIM2->PSC = 7199; // 72MHz / 7200 = 10 KHz
    // Set the Autoreload value
    TIM2->ARR = 4999;  // 10 KHz / 5000 = 2Hz
    TIM2->DIER |= TIM_DIER_UIE; // interrupt on update
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);

//  mainInitialize();

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

    // Infinite loop
    while (1) {
        GPIOC->ODR ^= GPIO_ODR_13;
        delay_ms(300);
//    mainCycle();
    }
#pragma clang diagnostic pop
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    // External oscillator (HSE): 24MHz
    // SYS_CLK = 72MHz
    // APB1 = 36MHz
    RCC->CR |= RCC_CR_HSEON;          // Enable HSE
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

uint32_t tim2count = 0;

void TIM2_IRQHandler() {
    tim2count++;
    TIM2->SR = (uint16_t) ~TIM_SR_UIF;
    GPIOC->ODR ^= GPIO_ODR_15;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
    }
#pragma clang diagnostic pop
}

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
