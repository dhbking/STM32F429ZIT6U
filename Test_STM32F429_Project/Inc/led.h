#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"

#define LED_GREEN_PIN	GPIO_PIN_0
#define LED_BLUE_PIN	GPIO_PIN_7
#define LED_RED_PIN		GPIO_PIN_14

#define LED_GREEN_GPIO	GPIOB
#define LED_BLUE_GPIO	GPIOB
#define LED_RED_GPIO	GPIOB

void Led_Init(void);
void Led_Show(void);

#endif
