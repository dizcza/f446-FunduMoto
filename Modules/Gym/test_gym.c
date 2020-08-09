/*
 * test_gym.c
 *
 *  Created on: Jan 26, 2020
 *      Author: dizcza
 */

#define TEST_GYM_EPS (1e-5)

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdint.h>

#include "test_gym.h"
#include "fundumoto.h"
#include "gym.h"


static uint8_t _is_close(float val, const float val_target) {
	val -= val_target;
	if (val < 0) {
		val = -val;
	}
	uint8_t res = (uint8_t) (val < TEST_GYM_EPS);
	return res;
}


void Test_GymMove() {
	Gym_Action action;

	action.left_wheel_vel = 1.f;
	action.right_wheel_vel = 1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	action.left_wheel_vel = -1.f;
	action.right_wheel_vel = -1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	action.left_wheel_vel = -1.f;
	action.right_wheel_vel = 1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	action.left_wheel_vel = 1.f;
	action.right_wheel_vel = -1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	action.left_wheel_vel = 1.f;
	action.right_wheel_vel = 0.5f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);
}


void Test_GymInfer() {
	// Current nnactor always predict 1.0 for the right wheel.
	Gym_Observation observation;
	Gym_Action action;
	ai_error inference_err;
	observation.servo_angle = 0.f;

	observation.dist_to_obstacle = 0.f;
	inference_err = Gym_Infer(&observation, &action);
	assert_param(action.left_wheel_vel < -0.969);
	assert_param(_is_close(action.right_wheel_vel, 1.f));
	assert_param(inference_err.type == AI_ERROR_NONE);

	observation.dist_to_obstacle = 1.f;
	inference_err = Gym_Infer(&observation, &action);
	assert_param(_is_close(action.left_wheel_vel, 1.f));
	assert_param(_is_close(action.right_wheel_vel, 1.f));
	assert_param(inference_err.type == AI_ERROR_NONE);
}
