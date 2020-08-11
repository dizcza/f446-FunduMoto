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


static void assert_isclose(float val, const float val_target) {
	assert_param(fabsf(val - val_target) < TEST_GYM_EPS);
}


void Test_GymMove() {
	Gym_Action action;

	// forward
	action.left_wheel_vel = 1.f;
	action.right_wheel_vel = 1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	// backward
	action.left_wheel_vel = -1.f;
	action.right_wheel_vel = -1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	// turn left
	action.left_wheel_vel = -1.f;
	action.right_wheel_vel = 1.f;
	FunduMoto_GymMove(&action);
	HAL_Delay(1000);

	// turn right
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
	// Current nnactor always predicts 1.0 for the right (second) wheel.
	Gym_Observation observation;
	observation.servo_angle = 0.f;
	Gym_Action action;
	ai_error inference_err;

	observation.dist_to_obstacle = 0.0001f;  // small dist
	inference_err = Gym_Infer(&observation, &action);
	assert_param(action.left_wheel_vel < -0.95f);
	assert_isclose(action.right_wheel_vel, 1.f);
	assert_param(inference_err.type == AI_ERROR_NONE);

	observation.dist_to_obstacle = 1.0f;
	inference_err = Gym_Infer(&observation, &action);
	assert_isclose(action.left_wheel_vel, 1.f);
	assert_isclose(action.right_wheel_vel, 1.f);
	assert_param(inference_err.type == AI_ERROR_NONE);
}
