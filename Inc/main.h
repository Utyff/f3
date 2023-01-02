#ifndef F3_MAIN_H
#define F3_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);

#define LED1_Pin 13u
#define LED1_GPIO_Port GPIOC
#define LED2_Pin 15u
#define LED2_GPIO_Port GPIOC
#define BTN1_Pin 14u
#define BTN1_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* F3_MAIN_H */
