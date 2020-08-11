/*
 * gym.h
 *
 *  Created on: Oct 20, 2019
 *      Author: Danylo Ulianych, d.ulianych@gmail.com
 */

#ifndef GYM_H_
#define GYM_H_

/**
 * These macros are set to match the Gym simulation
 * in which the network has been trained.
 */
#define GYM_SENSOR_DIST_MAX (200.f)  // cm
#define GYM_SERVO_ANGLE_MAX (30)     // degrees
#define GYM_SERVO_ANGLE_STEP (2)     // degrees


#include <ai_platform.h>

typedef struct Gym_Observation {
	float dist_to_obstacle;          // [0, 1] range
	float servo_angle;               // [0, 1] range
} Gym_Observation;

typedef struct Gym_Action {
	float left_wheel_vel;           // [-1, 1] range duty cycle
	float right_wheel_vel;          // [-1, 1] range duty cycle
} Gym_Action;

ai_error Gym_InitNetwork();
ai_error Gym_Infer(const Gym_Observation* obs, Gym_Action* action);
ai_error Gym_GetError();
ai_handle Gym_NetworkHandle();
void Gym_LogError(ai_error err, char* title);
void Gym_LogNetworkInfo();

#endif /* GYM_H_ */
