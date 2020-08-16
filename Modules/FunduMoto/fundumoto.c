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
#define SONAR_VEC_RANGE_SIZE (180 / SERVO_ANGLE_STEP + 1)

Fundu_Motor motorA = { .htim = &htim4, .direction_gpio = GPIOA, .direction_pin =
GPIO_PIN_6, .duty_cycle = 0U };
Fundu_Motor motorB = { .htim = &htim14, .direction_gpio = GPIOA,
		.direction_pin = GPIO_PIN_5, .duty_cycle = 0U };

volatile int32_t FunduMoto_MotorCycles = 0;
volatile uint32_t FunduMoto_SonarEchoUSec = 0U;  // microseconds

static RingBuffer_DMA ringbuf_rx;
static uint8_t rx_buf[FUNDUMOTO_RX_SIZE];

static uint8_t tx_buf[FUNDUMOTO_TX_SIZE];
static uint32_t tx_buf_count = 0;

/* Array for received commands */
static uint8_t rx_cmd[32];

static SonarVector m_sonar_vec[SONAR_MEDIAN_FILTER_SIZE_MAX];
static uint32_t m_angular_dist_events = 0U;
static int32_t m_servo_angle = 0;
static SonarVector m_last_sonar_vec;

static uint32_t m_sonar_max_dist = 400;
static uint32_t m_sonar_tolerance = 1;
static uint32_t m_sonar_median_filter_size = 5;

// autonomous mode
static int32_t m_servo_angle_step = GYM_SERVO_ANGLE_STEP;
static FunduMode m_mode = FUNDU_MODE_JOYSTICK;

static uint32_t FunduMoto_GetDutyCycle(float radius_norm);
static void FunduMoto_ProcessCommand();
static int8_t IsSonarVectorChanged(const SonarVector *vec);
static void ResetInternalState();
static void CopySonarVector(SonarVector *dest, const SonarVector *src);
static void TXWriteSonarVec(const SonarVector *vec, const uint8_t is_target);
static void FunduMoto_ChangeWheelVelocity(Fundu_Motor* motor, float velocity);

__STATIC_INLINE void ClampServoAngle(const int32_t left, const int32_t right) {
	if (m_servo_angle > right) {
		m_servo_angle = right;
	} else if (m_servo_angle < left) {
		m_servo_angle = left;
	}
}

static void ResetInternalState() {
	m_last_sonar_vec.servo_angle = 0;
	m_last_sonar_vec.sonar_dist = 0;
}

static void TXWriteSonarVec(const SonarVector *vec, const uint8_t is_target) {
	float dist_norm = vec->sonar_dist / (float) m_sonar_max_dist;
	tx_buf_count += sprintf((char *) tx_buf + tx_buf_count, "S%d,%03ld,%.3f\r\n",
			is_target, vec->servo_angle, dist_norm);
}

static HAL_StatusTypeDef TXSend() {
	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart4, tx_buf, tx_buf_count);
	tx_buf_count = 0;
	return status;
}

static void TXWriteFPS(const float fps) {
	tx_buf_count += sprintf((char *) tx_buf + tx_buf_count, "F%.2f\r\n", fps);
}


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
	RingBuffer_DMA_Init(&ringbuf_rx, huart4.hdmarx, rx_buf, FUNDUMOTO_RX_SIZE);
	/* Start UART4 DMA Reception */
	HAL_UART_Receive_DMA(&huart4, rx_buf, FUNDUMOTO_RX_SIZE);

	ResetInternalState();
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

void FunduMoto_UserMove(int32_t direction_angle, float velocity) {
	const uint32_t radius_dc = FunduMoto_GetDutyCycle(velocity);
	MotorDirection direction = (MotorDirection) (direction_angle > 0);
	if (direction_angle < 0) {
		direction_angle = -direction_angle;
	}

	// range [0, 180] contains 181 elements
	const float relative_scale = direction_angle * 1.f
			/ (181 - direction_angle);

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
	FunduMoto_MotorCycles =	(uint32_t) (MOTOR_MOVE_PERIOD * htim4.Init.Period);
}

void FunduMoto_GymMove(const Gym_Action* action) {
	FunduMoto_ChangeWheelVelocity(&motorA, action->left_wheel_vel);
	FunduMoto_ChangeWheelVelocity(&motorB, action->right_wheel_vel);
	FunduMoto_MotorCycles =	(uint32_t) (MOTOR_MOVE_PERIOD * htim4.Init.Period);
}


static void FunduMoto_ChangeWheelVelocity(Fundu_Motor* motor, float velocity) {
	MotorDirection direction = FORWARD;
	if (velocity < 0) {
		velocity = -velocity;
		direction = BACKWARD;
	}
	motor->duty_cycle = FunduMoto_GetDutyCycle(velocity);
	FunduMoto_SetDirection(motor, direction);
}


static void FunduMoto_ProcessCommand(const uint32_t rx_cmd_len) {
	if (rx_cmd_len == 0) {
		return;
	}
	ResetInternalState();
	switch (rx_cmd[0]) {
	case 'A':  // [A]utonomous
		;
		FunduMode mode = (FunduMode) atoi((const char*)rx_cmd + 1);
		FunduMoto_SetMode(mode);
		break;
	case 'M':  // [M]otor
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
		FunduMoto_UserMove(angle, velocity);
		break;
	case 'B':  // [B]uzzer
		// format: B<state:1d>
		if (rx_cmd_len != 2) {
			// invalid packet
			return;
		}
		GPIO_PinState buzzer_state = (GPIO_PinState) (rx_cmd[1] - '0');
		FunduMoto_Beep(buzzer_state);
		break;
	case 'S':  // [S]ervo
		// format: S<angle:3d>
		// angle in [-90, 90]
		if (rx_cmd_len != 4) {
			// invalid packet
			return;
		}
		m_servo_angle = atoi((char*) &rx_cmd[1]);
		ClampServoAngle(-90, 90);
		break;
	case 'D':  // set sonar max [D]ist, cm
		// format: D<dist:3d>
		if (rx_cmd_len != 4) {
			// invalid packet;
			return;
		}
		m_sonar_max_dist = atoi((char*) &rx_cmd[1]);
		break;
	case 'T':  // set sonar [T]olerance, cm
		// format: T<tol:1d>
		if (rx_cmd_len != 2) {
			// invalid packet
			return;
		}
		m_sonar_tolerance = atoi((char*) &rx_cmd[1]);
		break;
	case 'F':  // set sonar median [F]ilter size
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

void FunduMoto_ReadUART() {
	uint32_t rx_count = RingBuffer_DMA_Count(&ringbuf_rx);
	uint32_t rx_cmd_len = 0U;
	/* Process each byte individually */
	while (rx_count--) {
		/* Read out one byte from RingBuffer */
		uint8_t b = RingBuffer_DMA_GetByte(&ringbuf_rx);
		switch (b) {
		case '\r':
			// ignore \r
			break;
		case '\n':
			/* Terminate string with \0 and process the command. */
			rx_cmd[rx_cmd_len] = '\0';
			FunduMoto_ProcessCommand(rx_cmd_len);
			rx_cmd_len = 0U;
			break;
		default:
			rx_cmd[rx_cmd_len++] = b;
			break;
		}
	}
}

int32_t FunduMoto_GetServoAngle() {
	if (m_mode == FUNDU_MODE_AUTONOMOUS) {
		m_servo_angle += m_servo_angle_step;
		if (m_servo_angle <= -GYM_SERVO_ANGLE_MAX || m_servo_angle >= GYM_SERVO_ANGLE_MAX) {
			m_servo_angle_step *= -1;
		}
		ClampServoAngle(-GYM_SERVO_ANGLE_MAX, GYM_SERVO_ANGLE_MAX);
	}
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


static void CopySonarVector(SonarVector *dest, const SonarVector *src) {
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

void ReadSonarDist() {
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

void Gym_Update(const SonarVector* vec) {
	float servo_angle = (float) vec->servo_angle;  // [-30, 30]
	servo_angle = (servo_angle + GYM_SERVO_ANGLE_MAX) / (2 * GYM_SERVO_ANGLE_MAX);  // [0, 1]
	float dist_to_obstacle = vec->sonar_dist / GYM_SENSOR_DIST_MAX;
	if (dist_to_obstacle > 1.f) {
		dist_to_obstacle = 1.f;
	}
	Gym_Observation observation = {
			.dist_to_obstacle = dist_to_obstacle,
			.servo_angle = servo_angle,
	};
	Gym_Action action;
	Gym_Infer(&observation, &action);
	FunduMoto_GymMove(&action);
}


void FunduMoto_Update() {
	if (FunduMoto_SonarEchoUSec == SONAR_ECHO_USEC_IDLE) {
		return;
	}

	ReadSonarDist();

	SonarVector vec_median;
	if (GetFilteredSonarVector(&vec_median) != 0) {
		return;
	}

	if (m_mode == FUNDU_MODE_AUTONOMOUS) {
		Gym_Update(&vec_median);
	}

	if (!IsSonarVectorChanged(&vec_median)) {
		return;
	}

	TXWriteSonarVec(&vec_median, 0U);
	TXWriteFPS(FunduMoto_GetFPS());

	TXSend();

}

void FunduMoto_SetMode(FunduMode mode) {
	m_mode = mode;
	FunduMoto_MotorCycles = 0;
}

void FunduMoto_Beep(GPIO_PinState state) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state);
}
