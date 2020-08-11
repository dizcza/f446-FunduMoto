/*
 * fundumoto.h
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */

#ifndef FUNDUMOTO_H_
#define FUNDUMOTO_H_

/* --- DO NOT MODIFY THESE --- */
#define SONAR_TRIGGER_BURST_TICKS 5
#define SONAR_TICK_USEC 2
#define SONAR_SOUND_SPEED_INV 54  // 1e2 / (343 / 2 * 1e6) [usec/centimeters]
#define SERVO_90_DC 1500  // servo at 90° looks straight forward
#define SERVO_STEP_DC 10  // corresponds to 1°
#define ARG_SEPARATOR ','
#define SONAR_MEDIAN_FILTER_SIZE_MAX 10
/* ---- END DO NOT MODIFY ---- */

/* -- MODIFIABLE PARAMETERS -- */
#define MOTOR_MOVE_PERIOD (0.5f)

// Min normalized timer DC that drives the motor.
// Due to the friction and robot inertia, values below are
// not sufficient to drive the motor.
// Value between 0.0 and 1.0.
// Full DC is defined by htim4.Period.
#define DUTY_CYCLE_MIN_NORM (0.11f)

/* Array for DMA to save Rx bytes */
#define RINGBUF_RX_SIZE 256
/* ---- END MODIFIABLE ------- */


#include <stdint.h>
#include "stm32f446xx.h"
#include "tim.h"
#include "gym.h"


typedef struct Fundu_Motor {
	TIM_HandleTypeDef *htim;
	GPIO_TypeDef* direction_gpio;
	uint16_t direction_pin;
	uint32_t duty_cycle;
} Fundu_Motor;

typedef enum {
	BACKWARD = 0U,
	FORWARD = !BACKWARD
} MotorDirection;

typedef struct SonarVector {
	int32_t servo_angle;
	int32_t sonar_dist;
} SonarVector;

typedef enum {
	FUNDU_MODE_JOYSTICK = 0,
	FUNDU_MODE_AUTONOMOUS
} FunduMode;

extern Fundu_Motor motorA;
extern Fundu_Motor motorB;

__STATIC_INLINE void FunduMoto_SetDirection(const Fundu_Motor *motor, MotorDirection direction) {
	HAL_GPIO_WritePin(motor->direction_gpio, motor->direction_pin, direction);
}

void FunduMoto_Init();
void FunduMoto_ReadUART();
void FunduMoto_Update();
void FunduMoto_UserMove(int32_t direction_angle, float velocity);
void FunduMoto_GymMove(const Gym_Action* action);
int32_t FunduMoto_GetServoAngle();
void FunduMoto_SetMode(FunduMode mode);

#endif /* FUNDUMOTO_H_ */
