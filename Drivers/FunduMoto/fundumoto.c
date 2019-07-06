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

Fundu_Motor motorA = { .htim = &htim4, .direction_gpio = GPIOA, .direction_pin =
GPIO_PIN_6, .duty_cycle = 0U };
Fundu_Motor motorB = { .htim = &htim3, .direction_gpio = GPIOA, .direction_pin =
GPIO_PIN_5, .duty_cycle = 0U };

volatile int32_t Fundu_Motor_Cycles = 0;

RingBuffer_DMA rx_buf;
/* Array for DMA to save Rx bytes */
#define RINGBUF_RX_SIZE 256
uint8_t rx[RINGBUF_RX_SIZE];
/* Array for received commands */
int8_t cmd[128];
uint32_t icmd = 0;
/* Array for Tx messages */
uint8_t tx[100];
static uint32_t do_not_skip_bytes = 0U;

void FunduMoto_Init() {
	/* Init RingBuffer_DMA object */
	RingBuffer_DMA_Init(&rx_buf, huart4.hdmarx, rx, RINGBUF_RX_SIZE);
	/* Start UART4 DMA Reception */
	HAL_UART_Receive_DMA(&huart4, rx, RINGBUF_RX_SIZE);

}

uint32_t FunduMoto_GetDutyCycle(const int8_t radius) {
	if (radius == 0) {
		// avoid motor bounce (unstable state at DUTY_CYCLE_MIN)
		return 0U;
	}
	float radius_norm = radius * 1.f / VELOCITY_AMPLITUDE;
	// increase speed slowly ~ radius ^ 2
	radius_norm *= radius_norm;
	const uint32_t dc_min = DUTY_CYCLE_MIN_NORM * htim4.Init.Period;
	return dc_min + radius_norm * (htim4.Init.Period - dc_min);
}

void FunduMoto_SetDirection(const Fundu_Motor *motor, Motor_Direction direction) {
	HAL_GPIO_WritePin(motor->direction_gpio, motor->direction_pin, direction);
}

void FunduMoto_Move(int8_t angle_bin, int8_t radius) {
	sprintf((char *) tx, "[a%d, r%d] A=%lu, B=%lu\r\n", angle_bin, radius,
			motorA.duty_cycle, motorB.duty_cycle);
	HAL_UART_Transmit_IT(&huart4, tx, strlen((char *) tx));
	const uint32_t radius_dc = FunduMoto_GetDutyCycle(radius);
	Motor_Direction direction = (Motor_Direction) (angle_bin > 0);
	if (angle_bin < 0) {
		angle_bin = -angle_bin;
	}
	const float scale = angle_bin * 1.f / (ANGLE_BINS - angle_bin);
	uint32_t right_move, left_move;
	if (scale < 1.f) {
		right_move = (uint32_t) (scale * radius_dc);
		left_move = radius_dc;
	} else {
		right_move = radius_dc;
		left_move = (uint32_t) (1.f / scale * radius_dc);
	}
	motorA.duty_cycle = left_move;
	motorB.duty_cycle = right_move;
	FunduMoto_SetDirection(&motorA, direction);
	FunduMoto_SetDirection(&motorB, direction);
	Fundu_Motor_Cycles =
			(uint32_t) (FUNDU_MOTOR_MOVE_PERIOD * htim4.Init.Period);
}

void FunduMoto_ProcessCommand(int8_t buffer[], uint32_t length) {
	if (length == 0) {
		return;
	}
	switch (buffer[0]) {
	case 'M':
		// format: M<angle><radius>
		if (length == 3) {
			FunduMoto_Move(buffer[1], buffer[2]);
		}
		break;
	case 'B':
		// TODO implement buzzer
		break;
	}
}

void FunduMoto_Update() {
	uint32_t rx_count = RingBuffer_DMA_Count(&rx_buf);
	/* Process each byte individually */
	while (rx_count--) {
		/* Read out one byte from RingBuffer */
		uint8_t b = RingBuffer_DMA_GetByte(&rx_buf);
		if (do_not_skip_bytes) {
			cmd[icmd++] = (int8_t) b;
			do_not_skip_bytes--;
			continue;
		}
		switch (b) {
		case 'M':  // Move
			cmd[icmd++] = (int8_t) b;
			do_not_skip_bytes = 2U;
			break;
		case 'B':  // Buzzer
			cmd[icmd++] = (int8_t) b;
			do_not_skip_bytes = 0U;
			break;
		case '\r':
			// ignore \r
			break;
		case '\n':
			/* Terminate string with \0 and process the command. */
			cmd[icmd] = '\0';
			FunduMoto_ProcessCommand(cmd, icmd);
			icmd = 0;
			break;
		default:
			// we should never enter this block
			assert_param(0U);
			cmd[icmd++] = (int8_t) b;
			break;
		}
	}
}
