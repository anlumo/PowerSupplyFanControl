/*
 * power_led.c
 *
 *  Created on: 26 Jun 2017
 *      Author: Andreas Monitzer
 */


#include <stdint.h>
#include "power_led.h"
#include "main.h"
#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim16;

void setPowerLEDRatio(uint8_t ratio) {
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, ratio);
}

void setPowerLEDSpeed(uint16_t speed) {
	__HAL_TIM_SET_PRESCALER(&htim16, speed);
}
