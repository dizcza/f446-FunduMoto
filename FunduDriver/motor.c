/*
 * motor.c
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */


#include "fundumoto.h"
#include "tim.h"

Fundu_Motor motorA = {.htim=&htim4, .direction_gpio=GPIOA, .direction_pin=GPIO_PIN_6, .duty_cycle=0U};
Fundu_Motor motorB = {.htim=&htim3, .direction_gpio=GPIOA, .direction_pin=GPIO_PIN_5, .duty_cycle=0U};

volatile int32_t Fundu_Motor_Cycles = 0;


void Fundu_Init() {
	assert_param(DUTY_CYCLE_MIN <= htim4.Init.Period);
}

uint32_t Fundu_GetDutyCycle(const int8_t radius) {
	if (radius == 0) {
		// avoid motor bounce (unstable state at DUTY_CYCLE_MIN)
		return 0U;
	}
	const float radius_norm = radius * 1.f / VELOCITY_AMPLITUDE;
	return DUTY_CYCLE_MIN + radius_norm * (htim4.Init.Period - DUTY_CYCLE_MIN);
}


void Fundu_Motor_SetDirection(const Fundu_Motor *motor, Motor_Direction direction) {
	HAL_GPIO_WritePin(motor->direction_gpio, motor->direction_pin, direction);
}
