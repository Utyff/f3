#include <stm32f3xx.h>
#include "main.h"
#include <_main.h>
#include <delay.h>


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
    // Disable the Channel 1: Reset the CC1E Bit
    TIM1->CCER &= ~TIM_CCER_CC1E;
    // Select the Output Compare Mode
    TIM1->CCMR1 |= ((uint32_t) TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    // Set the Capture Compare Register value
    TIM1->CCR1 = 4000;
    // Set the Preload enable bit for channel1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    // Configure the Output Fast mode
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1FE;
    // Enable the Capture compare channel
    TIM1->CCER |= TIM_CCER_CC1E;
    // Enable the main output
    TIM1->BDTR |= TIM_BDTR_MOE;
    // Enable the Peripheral
    TIM1->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__)  (((__PCLK__) + ((__BAUD__)/2U)) / (__BAUD__))

void USART1_init() {
    // set PA9 PA10 for USART1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // high speed PA9 PA10
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;
    // AF7 for USART1 signals for PA9 PA10
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2)) | (7u << (1 * 4u)) | (7u << (2 * 4u));
    // Select AF mode (10) on PA9 PA10
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10)) | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;

    // Initialize TIM1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->CFGR3 &= ~RCC_CFGR3_USART1SW; // PCLK clock source for USART1

    // disable USART1
    USART1->CR1 &= ~USART_CR1_UE;
    // set baud rate
    USART1->BRR = UART_DIV_SAMPLING16(72000000, 115200);
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE; // enable USART and USART transmit. USART_CR1_RE for receive
//    USART1->CR1 |= USART_CR1_RXNEIE; // enable receive interrupt

    USART1->ICR |= USART_ICR_TCCF; // clear TC flag
    USART1->CR1 |= USART_CR1_TCIE; // enable TC interrupt

    NVIC_EnableIRQ(USART1_IRQn);   // enable USART1 interrupt
}

void _strcpy(uint8_t *dst, const uint8_t *src) {
    int i = 0;
    do {
        dst[i] = src[i];
    } while (src[i++] != 0);
}

uint8_t send = 0;
uint8_t string2send[200] = "";

void prints(const char *str) {
    // wait till end current transmission
    while (send != 0);

    _strcpy(string2send, (uint8_t *) str);
    // start USART transmission. Will initiate TC if TXE
    USART1->TDR = string2send[send];
    send = 1;
}

/**
  * @brief  The application entry point.
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
    USART1_init();
    prints("Hello !\n\r");

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

    while (1) {
        mainCycle();
    }

#pragma clang diagnostic pop
}

/**
  * @brief System Clock Configuration
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

uint16_t txCount = 0;

void USART1_IRQHandler(void) {
//    if (USART1->ISR & USART_CR1_RXNEIE) { // RXNEIE - data received
//        tt = USART1->RDR;
//    }

    if (USART1->ISR & USART_ISR_TC) {
        txCount++;
        if (string2send[send] == 0) {
            send = 0;
            USART1->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
            return;
        } else {
            // fill TDR with a new char and clear transfer complete flag
            USART1->TDR = string2send[send++];
        }
    }
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
