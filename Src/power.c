/*
 * power.c
 *
 *  Created on: 9 Jun 2017
 *      Author: Andreas Monitzer
 */

#include "power.h"
#include "main.h"
#include "stm32f0xx_hal.h"

void setPowerState(bool powerState) {
	HAL_GPIO_WritePin(PS_ON_GPIO_Port, PS_ON_Pin, powerState?GPIO_PIN_SET:GPIO_PIN_RESET);
}

bool getPowerState(void) {
	return HAL_GPIO_ReadPin(PS_ON_GPIO_Port, PS_ON_Pin) == GPIO_PIN_SET;
}
