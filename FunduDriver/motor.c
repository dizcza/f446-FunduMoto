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
}

uint32_t Fundu_GetDutyCycle(const int8_t radius) {
	if (radius == 0) {
		// avoid motor bounce (unstable state at DUTY_CYCLE_MIN)
		return 0U;
	}
	float radius_norm = radius * 1.f / VELOCITY_AMPLITUDE;
	// increase speed slowly ~ radius ^ 2
	radius_norm *= radius_norm;
	const uint32_t dc_min = DUTY_CYCLE_MIN_NORM * htim4.Init.Period;
	return dc_min + radius_norm * (htim4.Init.Period - dc_min);
}


void Fundu_Motor_SetDirection(const Fundu_Motor *motor, Motor_Direction direction) {
	HAL_GPIO_WritePin(motor->direction_gpio, motor->direction_pin, direction);
}


void FunduMoto_Process(int8_t buffer[], uint32_t length) {
	int8_t angle_bin = buffer[0];
	int8_t radius = buffer[1];
	const uint32_t radius_dc = Fundu_GetDutyCycle(radius);
	Motor_Direction direction = (Motor_Direction) (angle_bin > 0);
	if (angle_bin < 0) {
		angle_bin = -angle_bin;
	}
	const float scale = angle_bin * 1.f / (ANGLE_BINS - angle_bin);
	uint32_t right_move, left_move;
	if (scale < 1.f) {
		right_move = (uint32_t) (scale * radius_dc);
		left_move = radius_dc;
	} else {
		right_move = radius_dc;
		left_move = (uint32_t) (1.f / scale * radius_dc);
	}
	motorA.duty_cycle = left_move;
	motorB.duty_cycle = right_move;
	Fundu_Motor_SetDirection(&motorA, direction);
	Fundu_Motor_SetDirection(&motorB, direction);
	Fundu_Motor_Cycles = (uint32_t) (FUNDU_MOTOR_MOVE_PERIOD * htim4.Init.Period);
}
