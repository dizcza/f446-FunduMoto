/*
 * fundumoto.h
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */

#ifndef FUNDUMOTO_H_
#define FUNDUMOTO_H_

#include <stdint.h>
#include "tim.h"


#define FUNDU_MOTOR_CYCLES 100

typedef struct Fundu_Motor {
	TIM_HandleTypeDef *htim;
} Fundu_Motor;

void Fundu_Init();
void Fundu_Forward();

#endif /* FUNDUMOTO_H_ */
