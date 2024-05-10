#pragma once

#include "stm32f7xx_hal.h"

#define BLUE_LED_Pin GPIO_PIN_7
#define RED_LED_Pin GPIO_PIN_14

extern UART_HandleTypeDef huart3;
extern int errorActive;
