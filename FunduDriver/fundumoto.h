/*
 * fundumoto.h
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */

#ifndef FUNDUMOTO_H_
#define FUNDUMOTO_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "tim.h"

/* --- DO NOT MODIFY THESE --- */
#define VELOCITY_AMPLITUDE 100
#define ANGLE_BINS 18
/* ---- END DO NOT MODIFY ---- */

/* -- MODIFIABLE PARAMETERS -- */
#define FUNDU_MOTOR_MOVE_PERIOD (0.5f)

// Min normalized timer DC that drives the motor.
// Value between 0.0 and 1.0.
// Full DC is defined by htim4.Period.
#define DUTY_CYCLE_MIN_NORM (0.11f)
/* ---- END MODIFIABLE ------- */


typedef struct Fundu_Motor {
	TIM_HandleTypeDef *htim;
	GPIO_TypeDef* direction_gpio;
	uint16_t direction_pin;
	uint32_t duty_cycle;
} Fundu_Motor;

typedef enum {
	BACKWARD = 0U,
	FORWARD = !BACKWARD
} Motor_Direction;

extern Fundu_Motor motorA;
extern Fundu_Motor motorB;

volatile int32_t Fundu_Motor_Cycles;


void Fundu_Init();
void Fundu_Motor_SetDirection(const Fundu_Motor *motor, Motor_Direction direction);
uint32_t Fundu_GetDutyCycle(const int8_t radius);
void FunduMoto_Process(int8_t buffer[], uint32_t length);

#endif /* FUNDUMOTO_H_ */
