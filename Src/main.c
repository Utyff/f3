#include <stdio.h>
#include <stm32f3xx.h>
#include <_main.h>


void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  */
int main(void) {
    // Configure the system clock
    SystemClock_Config();
    mainInitialize();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        mainCycle();
    }
#pragma clang diagnostic pop
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void) {
    // External oscillator (HSE) = 8MHz
    // SYS_CLK = 72MHz
    // APB1 = 36MHz
    RCC->CR |= RCC_CR_HSEON;           // Enable HSE
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    RCC->CFGR |= RCC_CFGR_PLLMUL9;     // PLL MUL = x9, SYS_CLK=72MHz (x3 for 24MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 Prescaler = 2
    RCC->CFGR |= RCC_CFGR_PLLSRC;      // PLL source = HSE

    FLASH->ACR |= FLASH_ACR_LATENCY_1; // Two wait states

    RCC->CR |= RCC_CR_PLLON;           // Enable and wait PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_SW_PLL;      // Select PLL as system clock

//    RCC_HSICmd(DISABLE);//Disable HSI
    SystemCoreClockUpdate();
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
