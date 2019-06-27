/*
 * motor.c
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */


#include "fundumoto.h"

Fundu_Motor motorA;
Fundu_Motor motorB;

volatile int32_t Fundu_Motor_Cycles = 0;
volatile uint32_t Fundu_Motor_DC = 0U;


void Fundu_Init() {
	motorA.htim = &htim4;
	motorB.htim = &htim3;
}


void Fundu_Forward() {
	Fundu_Motor_Cycles = FUNDU_MOTOR_CYCLES;
	Fundu_Motor_DC = 20U;
}
