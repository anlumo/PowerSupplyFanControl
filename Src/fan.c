/*
 * fan.c
 *
 *  Created on: May 14, 2017
 *      Author: anlumo
 */

#include <stdint.h>
#include "fan.h"
#include "main.h"
#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim1;

void setFanSpeed(uint8_t speed) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}
