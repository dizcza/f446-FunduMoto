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
#include <math.h>

#define SONAR_ECHO_USEC_IDLE 0U

Fundu_Motor motorA = { .htim = &htim4, .direction_gpio = GPIOA, .direction_pin =
GPIO_PIN_6, .duty_cycle = 0U };
Fundu_Motor motorB = { .htim = &htim14, .direction_gpio = GPIOA,
		.direction_pin = GPIO_PIN_5, .duty_cycle = 0U };

volatile int32_t FunduMoto_MotorCycles = 0;
volatile uint32_t FunduMoto_SonarEchoUSec = 0U;  // microseconds

RingBuffer_DMA rx_buf_dma;
uint8_t rx[RINGBUF_RX_SIZE];
uint8_t tx[32];

/* Array for received commands */
uint8_t rx_cmd[32];
uint32_t rx_cmd_len = 0U;

static SonarVector m_sonar_vec[SONAR_MEDIAN_FILTER_SIZE_MAX];
static uint32_t m_angular_dist_events = 0U;
static volatile int32_t m_servo_angle = 0;
static SonarVector m_last_sonar_vec;

static uint32_t m_sonar_max_dist = 400;
static uint32_t m_sonar_tolerance = 1;
static uint32_t m_sonar_median_filter_size = 5;

static uint32_t FunduMoto_GetDutyCycle(float radius_norm);
static void FunduMoto_ProcessCommand();
static int8_t IsSonarVectorChanged(const SonarVector *vec);
static void ResetInternalState();

void FunduMoto_Init() {
	assert_param(SONAR_TRIGGER_BURST_TICKS * SONAR_TICK_USEC == 10);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1); // motorA (handles interrupts for both motors)
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);    // motorB
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);  // servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // sonar

	// Init default PWM values
	htim1.Instance->CCR1 = SONAR_TRIGGER_BURST_TICKS;
	htim3.Instance->CCR2 = SERVO_90_DC;

	/* Init RingBuffer_DMA object */
	RingBuffer_DMA_Init(&rx_buf_dma, huart4.hdmarx, rx, RINGBUF_RX_SIZE);
	/* Start UART4 DMA Reception */
	HAL_UART_Receive_DMA(&huart4, rx, RINGBUF_RX_SIZE);

	ResetInternalState();
}

static void ResetInternalState() {
	m_last_sonar_vec.servo_angle = 0;
	m_last_sonar_vec.sonar_dist = 0;
}

static uint32_t FunduMoto_GetDutyCycle(float radius_norm) {
	if (radius_norm < 1e-2) {
		// avoid motor bounce (unstable state at DUTY_CYCLE_MIN)
		return 0U;
	}
	// increase speed slowly ~ radius ^ 2
	radius_norm *= radius_norm;
	const uint32_t dc_min = (uint32_t) (DUTY_CYCLE_MIN_NORM * htim4.Init.Period);
	return dc_min + radius_norm * (htim4.Init.Period - dc_min);
}

void FunduMoto_Move(int32_t direction_angle, float velocity) {
	const uint32_t radius_dc = FunduMoto_GetDutyCycle(velocity);
	MotorDirection direction = (MotorDirection) (direction_angle > 0);
	if (direction_angle < 0) {
		direction_angle = -direction_angle;
	}

	// +1 to avoid division by zero
	const float relative_scale = direction_angle * 1.f / (180 - direction_angle + 1);

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
	FunduMoto_MotorCycles = (uint32_t) (MOTOR_MOVE_PERIOD
			* htim4.Init.Period);
}

static void FunduMoto_ProcessCommand() {
	if (rx_cmd_len == 0) {
		return;
	}
	ResetInternalState();
	switch (rx_cmd[0]) {
	case 'M':  // Motor
		// format: M<angle:4d>,<velocity:.2f>
		// angle in [-180, 180]
		// velocity in [0, 1]
		if (rx_cmd_len != 10 || rx_cmd[5] != ARG_SEPARATOR) {
			// invalid packet
			return;
		}
		char angle_str[4] = { rx_cmd[1], rx_cmd[2], rx_cmd[3], rx_cmd[4] };
		int32_t angle = atoi(angle_str);
		float velocity = atof((char*) &rx_cmd[6]);
		FunduMoto_Move(angle, velocity);
		break;
	case 'B':  // Buzzer
		// format: B<state:1d>
		if (rx_cmd_len != 2) {
			// invalid packet
			return;
		}
		GPIO_PinState buzzer_state = (GPIO_PinState) (rx_cmd[1] - '0');
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, buzzer_state);
		break;
	case 'S':  // Servo
		// format: S<angle:3d>
		// angle in [-90, 90]
		if (rx_cmd_len != 4) {
			// invalid packet
			return;
		}
		m_servo_angle = atoi((char*) &rx_cmd[1]);
		if (m_servo_angle > 90) {
			m_servo_angle = 90;
		} else if (m_servo_angle < -90) {
			m_servo_angle = -90;
		}
		break;
	case 'D':  // set sonar max Dist, cm
		// format: D<dist:3d>
		if (rx_cmd_len != 4) {
			// invalid packet;
			return;
		}
		m_sonar_max_dist = atoi((char*) &rx_cmd[1]);
		break;
	case 'T':  // set sonar Tolerance, cm
		// format: T<tol:1d>
		if (rx_cmd_len != 2) {
			// invalid packet
			return;
		}
		m_sonar_tolerance = atoi((char*) &rx_cmd[1]);
		break;
	case 'F':  // set sonar median Filter size
		// format: F<size:1d>
		if (rx_cmd_len != 2) {
			// invalid packet
			return;
		}
		m_sonar_median_filter_size = atoi((char*) &rx_cmd[1]);
		if (m_sonar_median_filter_size < 1) {
			m_sonar_median_filter_size = 1;
		} else if (m_sonar_median_filter_size > SONAR_MEDIAN_FILTER_SIZE_MAX) {
			m_sonar_median_filter_size = SONAR_MEDIAN_FILTER_SIZE_MAX;
		}
		break;
	}
}

void FunduMoto_Update() {
	uint32_t rx_count = RingBuffer_DMA_Count(&rx_buf_dma);
	/* Process each byte individually */
	while (rx_count--) {
		/* Read out one byte from RingBuffer */
		uint8_t b = RingBuffer_DMA_GetByte(&rx_buf_dma);
		switch (b) {
		case '\r':
			// ignore \r
			break;
		case '\n':
			/* Terminate string with \0 and process the command. */
			rx_cmd[rx_cmd_len] = '\0';
			FunduMoto_ProcessCommand();
			rx_cmd_len = 0U;
			break;
		default:
			rx_cmd[rx_cmd_len++] = b;
			break;
		}
	}
}

int32_t FunduMoto_GetServoAngle() {
	return m_servo_angle;
}

int ServoVector_Comparator(const void *a, const void *b) {
	return ((SonarVector*) a)->sonar_dist - ((SonarVector*) b)->sonar_dist;
}

int8_t GetFilteredSonarVector(SonarVector *vec_filtered) {
	if (m_sonar_median_filter_size < 1) {
		// invalid filter size
		return 1;
	}
	if (m_angular_dist_events < m_sonar_median_filter_size) {
		// not enough points
		return 1;
	}
	SonarVector vec_sorted[m_sonar_median_filter_size];
	memcpy(vec_sorted, m_sonar_vec, sizeof(vec_sorted));
	qsort(vec_sorted, m_sonar_median_filter_size, sizeof(SonarVector),
			ServoVector_Comparator);
	uint32_t median_id = m_sonar_median_filter_size >> 1U;
	vec_filtered->servo_angle = vec_sorted[median_id].servo_angle;
	vec_filtered->sonar_dist = vec_sorted[median_id].sonar_dist;
	return 0;
}

void UpdateSonarDist() {
	uint32_t dist_cm = FunduMoto_SonarEchoUSec / SONAR_SOUND_SPEED_INV;
	int32_t servo_angle = (((int32_t) htim3.Instance->CCR2) - SERVO_90_DC)
			/ SERVO_STEP_DC;
	if (dist_cm > m_sonar_max_dist) {
		dist_cm = m_sonar_max_dist;
	}
	SonarVector *angular_dist = &m_sonar_vec[m_angular_dist_events
			% m_sonar_median_filter_size];
	angular_dist->servo_angle = servo_angle;
	angular_dist->sonar_dist = dist_cm;
	m_angular_dist_events++;
	FunduMoto_SonarEchoUSec = SONAR_ECHO_USEC_IDLE;
}

void CopySonarVector(SonarVector *dest, const SonarVector *src) {
	dest->servo_angle = src->servo_angle;
	dest->sonar_dist = src->sonar_dist;
}

static int8_t IsSonarVectorChanged(const SonarVector *vec) {
	if (vec->servo_angle != m_last_sonar_vec.servo_angle) {
		CopySonarVector(&m_last_sonar_vec, vec);
		return 1;
	}
	int32_t dist_diff = vec->sonar_dist - m_last_sonar_vec.sonar_dist;
	if (dist_diff < 0) {
		dist_diff = -dist_diff;
	}
	if (dist_diff > m_sonar_tolerance) {
		CopySonarVector(&m_last_sonar_vec, vec);
		return 1;
	}
	return 0;
}

void FunduMoto_SendSonarDist() {
	if (FunduMoto_SonarEchoUSec != SONAR_ECHO_USEC_IDLE) {
		UpdateSonarDist();

		SonarVector vec_median;
		if (GetFilteredSonarVector(&vec_median) != 0) {
			return;
		}

		if (!IsSonarVectorChanged(&vec_median)) {
			return;
		}

		float dist_norm = vec_median.sonar_dist / (float) m_sonar_max_dist;
		sprintf((char *) tx, "S%03ld,%.3f\r\n", vec_median.servo_angle,
				dist_norm);
		HAL_UART_Transmit_IT(&huart4, tx, strlen((char *) tx));
	}
}
