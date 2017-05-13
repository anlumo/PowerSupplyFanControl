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
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
