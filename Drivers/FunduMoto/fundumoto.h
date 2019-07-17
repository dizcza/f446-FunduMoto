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
#define SONAR_TRIGGER_BURST_TICKS 5
#define SONAR_TICK_USEC 2
#define SONAR_SOUND_SPEED_INV 54  // 1e2 / (343 / 2 * 1e6) [usec/centimeters]
#define SERVO_90 1500
#define SERVO_STEP 10
#define ARG_SEPARATOR ','
/* ---- END DO NOT MODIFY ---- */

/* -- MODIFIABLE PARAMETERS -- */
#define FUNDU_MOTOR_MOVE_PERIOD (0.5f)

// Min normalized timer DC that drives the motor.
// Value between 0.0 and 1.0.
// Full DC is defined by htim4.Period.
#define DUTY_CYCLE_MIN_NORM (0.11f)

#define SONAR_MAX_DIST 400  // centimeters
#define SONAR_MEDIAN_FILTER_SIZE 3
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

typedef struct Fundu_SonarVector {
	int32_t servo_angle;
	int32_t sonar_dist;
} Fundu_SonarVector;

extern Fundu_Motor motorA;
extern Fundu_Motor motorB;

__STATIC_INLINE void FunduMoto_SetDirection(const Fundu_Motor *motor, Motor_Direction direction) {
	HAL_GPIO_WritePin(motor->direction_gpio, motor->direction_pin, direction);
}

void FunduMoto_Init();
uint32_t FunduMoto_GetDutyCycle(float radius_norm);
void FunduMoto_ProcessCommand();
void FunduMoto_Update();
void FunduMoto_SendSonarDist();
void FunduMoto_Move(int32_t angle_bin, float radius_norm);
int32_t FunduMoto_GetServoAngle();


#endif /* FUNDUMOTO_H_ */
