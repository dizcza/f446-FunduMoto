/*
 * gym.h
 *
 *  Created on: Oct 20, 2019
 *      Author: Danylo Ulianych, d.ulianych@gmail.com
 */

#ifndef GYM_H_
#define GYM_H_

#define GYM_SENSOR_DIST_MAX (200.f)  // cm
#define GYM_SERVO_ANGLE_MAX (30)     // degrees
#define GYM_SERVO_ANGLE_STEP (2)     // degrees


#include <ai_platform.h>

typedef struct Gym_Observation {
	float dist_to_obstacle;
	float servo_angle;
} Gym_Observation;

typedef struct Gym_Action {
	// normalized wheels velocity
	float left_wheel_vel;
	float right_wheel_vel;
} Gym_Action;

ai_error Gym_InitNetwork();
ai_error Gym_Infer(const Gym_Observation* obs, Gym_Action* action);
ai_error Gym_GetError();
void Gym_LogError(ai_error err, char* title);
void Gym_LogNetworkInfo();

#endif /* GYM_H_ */
