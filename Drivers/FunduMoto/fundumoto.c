/*
 * motor.c
 *
 *  Created on: Jun 26, 2019
 *      Author: Danylo Ulianych
 */

#include "tim.h"
#include "fundumoto.h"
#include "usart.h"
#include "ringbuffer_dma.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define SONAR_ECHO_USEC_IDLE (-1)

Fundu_Motor motorA = { .htim = &htim4, .direction_gpio = GPIOA, .direction_pin =
GPIO_PIN_6, .duty_cycle = 0U };
Fundu_Motor motorB = { .htim = &htim14, .direction_gpio = GPIOA,
		.direction_pin = GPIO_PIN_5, .duty_cycle = 0U };

volatile int32_t FunduMoto_MotorCycles = 0;
volatile int32_t FunduMoto_SonarEchoUSec = 0U;  // microseconds

RingBuffer_DMA rx_buf;
/* Array for DMA to save Rx bytes */
#define RINGBUF_RX_SIZE 256
uint8_t rx[RINGBUF_RX_SIZE];
/* Array for received commands */
uint8_t cmd[128];
uint32_t cmd_len = 0U;
/* Array for Tx messages */
uint8_t tx[100];

static Fundu_SonarVector m_sonar_vec[SONAR_MEDIAN_FILTER_SIZE];
static uint32_t m_angular_dist_events = 0U;

static volatile int32_t m_servo_angle = 0;

void FunduMoto_Init() {
	assert_param(SONAR_TRIGGER_BURST_TICKS * SONAR_TICK_USEC == 10);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1); // motorA (handles interrupts for both motors)
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);    // motorB
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);  // servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // sonar
	htim1.Instance->CCR2 = SONAR_TRIGGER_BURST_TICKS;

	/* Init RingBuffer_DMA object */
	RingBuffer_DMA_Init(&rx_buf, huart4.hdmarx, rx, RINGBUF_RX_SIZE);
	/* Start UART4 DMA Reception */
	HAL_UART_Receive_DMA(&huart4, rx, RINGBUF_RX_SIZE);

}

uint32_t FunduMoto_GetDutyCycle(float radius_norm) {
	if (radius_norm < 1e-2) {
		// avoid motor bounce (unstable state at DUTY_CYCLE_MIN)
		return 0U;
	}
	// increase speed slowly ~ radius ^ 2
	radius_norm *= radius_norm;
	const uint32_t dc_min = (uint32_t) (DUTY_CYCLE_MIN_NORM * htim4.Init.Period);
	return dc_min + radius_norm * (htim4.Init.Period - dc_min);
}

void FunduMoto_Move(int32_t angle, float radius_norm) {
	const uint32_t radius_dc = FunduMoto_GetDutyCycle(radius_norm);
	Motor_Direction direction = (Motor_Direction) (angle > 0);
	if (angle < 0) {
		angle = -angle;
	}

	// +1 to avoid division by zero
	const float relative_scale = angle * 1.f / (90 - angle + 1);

	uint32_t right_move, left_move;
	if (relative_scale < 1.f) {
		right_move = (uint32_t) (relative_scale * radius_dc);
		left_move = radius_dc;
	} else {
		right_move = radius_dc;
		left_move = (uint32_t) (1.f / relative_scale * radius_dc);
	}
	motorA.duty_cycle = left_move;
	motorB.duty_cycle = right_move;
	FunduMoto_SetDirection(&motorA, direction);
	FunduMoto_SetDirection(&motorB, direction);
	FunduMoto_MotorCycles = (uint32_t) (FUNDU_MOTOR_MOVE_PERIOD
			* htim4.Init.Period);
}

void FunduMoto_ProcessCommand() {
	if (cmd_len == 0) {
		return;
	}
	switch (cmd[0]) {
	case 'M':  // Motor
		// format: M<angle:3d>,<radius:2d>
		// angle in [-90, 90]
		// radius in [0, VELOCITY_AMPLITUDE]
		if (cmd[4] != ARG_SEPARATOR) {
			// invalid packet
			return;
		}
		char angle_str[4] = { cmd[1], cmd[2], cmd[3] };
		int32_t angle = atoi(angle_str);
		float radius_norm = atof((char*) &cmd[5]);
		FunduMoto_Move(angle, radius_norm);
		break;
	case 'B':  // Buzzer
		// TODO implement buzzer
		break;
	case 'S':  // Servo
		// format: S<angle:3d>
		// angle in [-90, 90]
		m_servo_angle = atoi((char*) &cmd[1]);
		if (m_servo_angle > 90) {
			m_servo_angle = 90;
		} else if (m_servo_angle < -90) {
			m_servo_angle = -90;
		}
		break;
	}
}

void FunduMoto_Update() {
	uint32_t rx_count = RingBuffer_DMA_Count(&rx_buf);
	/* Process each byte individually */
	while (rx_count--) {
		/* Read out one byte from RingBuffer */
		uint8_t b = RingBuffer_DMA_GetByte(&rx_buf);
		switch (b) {
		case '\r':
			// ignore \r
			break;
		case '\n':
			/* Terminate string with \0 and process the command. */
			cmd[cmd_len] = '\0';
			FunduMoto_ProcessCommand();
			cmd_len = 0U;
			break;
		default:
			cmd[cmd_len++] = b;
			break;
		}
	}
}

int32_t FunduMoto_GetServoAngle() {
	return m_servo_angle;
}

void FunduMoto_SendSonarDist() {
	if (FunduMoto_SonarEchoUSec != SONAR_ECHO_USEC_IDLE) {
		int32_t dist_cm = FunduMoto_SonarEchoUSec / SONAR_SOUND_SPEED_INV;
		FunduMoto_SonarEchoUSec = SONAR_ECHO_USEC_IDLE;
		int32_t servo_angle = (((int32_t) htim3.Instance->CCR2) - SERVO_90)
				/ SERVO_STEP;
		if (dist_cm > SONAR_MAX_DIST) {
			dist_cm = SONAR_MAX_DIST;
		}
		float dist_norm = dist_cm / (float) SONAR_MAX_DIST;
		Fundu_SonarVector *angular_dist = &m_sonar_vec[m_angular_dist_events
				% SONAR_MEDIAN_FILTER_SIZE];
		angular_dist->servo_angle = servo_angle;
		angular_dist->sonar_dist = dist_cm;
		m_angular_dist_events++;
		sprintf((char *) tx, "S%03ld,%.2f\r\n", servo_angle, dist_norm);
		HAL_UART_Transmit_IT(&huart4, tx, strlen((char *) tx));
	}
}
