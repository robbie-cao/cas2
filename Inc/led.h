#ifndef __LED_H
#define __LED_H

#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"


/* Pin definitions */
#define LED_PORT			GPIOF
#define LED_PIN_LEFT		GPIO_PIN_7
#define LED_PIN_CENTER		GPIO_PIN_8
#define LED_PIN_RIGHT		GPIO_PIN_6

#define LED_CLK_ENABLE()	__HAL_RCC_GPIOF_CLK_ENABLE()
#define LED_CLK_DISABLE()	__HAL_RCC_GPIOF_CLK_DISABLE()

#define LED_LEFT_ON()		HAL_GPIO_WritePin(LED_PORT, LED_PIN_LEFT, GPIO_PIN_RESET)
#define LED_LEFT_OFF()		HAL_GPIO_WritePin(LED_PORT, LED_PIN_LEFT, GPIO_PIN_SET)
#define LED_LEFT_TOGGLE()	HAL_GPIO_TogglePin(LED_PORT, LED_PIN_LEFT)

#define LED_RIGHT_ON()		HAL_GPIO_WritePin(LED_PORT, LED_PIN_RIGHT, GPIO_PIN_RESET)
#define LED_RIGHT_OFF()		HAL_GPIO_WritePin(LED_PORT, LED_PIN_RIGHT, GPIO_PIN_SET)
#define LED_RIGHT_TOGGLE()	HAL_GPIO_TogglePin(LED_PORT, LED_PIN_RIGHT)

#define LED_CENTER_ON()		HAL_GPIO_WritePin(LED_PORT, LED_PIN_CENTER, GPIO_PIN_RESET)
#define LED_CENTER_OFF()	HAL_GPIO_WritePin(LED_PORT, LED_PIN_CENTER, GPIO_PIN_SET)
#define LED_CENTER_TOGGLE()	HAL_GPIO_TogglePin(LED_PORT, LED_PIN_CENTER)


#endif




