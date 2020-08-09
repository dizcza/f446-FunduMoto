/*
 * gym.c
 *
 *  Created on: Oct 20, 2019
 *      Author: Danylo Ulianych, d.ulianych@gmail.com
 */


#include "gym.h"

#include <cmsis_gcc.h>
#include <nnactor.h>
#include <nnactor_data.h>
#include <stdio.h>
#include <sys/_stdint.h>

#define SINGLE_PASS_BATCH_SIZE 1

static ai_float m_input_data[AI_NNACTOR_IN_1_SIZE];
static ai_float m_output_data[AI_NNACTOR_OUT_1_SIZE];

ai_handle m_network = AI_HANDLE_NULL;

/* Global buffer to handle the activations data buffer - R/W data */
AI_ALIGNED(4)
static ai_u8 m_activations[AI_NNACTOR_DATA_ACTIVATIONS_SIZE];

ai_error Gym_InitNetwork() {
	ai_error err;

	err = ai_nnactor_create(&m_network, AI_NNACTOR_DATA_CONFIG);
	if (err.type != AI_ERROR_NONE) {
		Gym_LogError(err, "ai_nnactor_create");
		ai_nnactor_destroy(m_network);
		return err;
	}

	ai_buffer params_buffer =
	AI_NNACTOR_DATA_WEIGHTS(ai_nnactor_data_weights_get());
	ai_buffer activations_buffer = AI_NNACTOR_DATA_ACTIVATIONS(m_activations);
	const ai_network_params params = { params_buffer, activations_buffer };
	if (!ai_nnactor_init(m_network, &params)) {
		err = ai_nnactor_get_error(m_network);
		Gym_LogError(err, "ai_nnactor_init");
		ai_nnactor_destroy(m_network);
	}

	return err;
}

ai_error Gym_GetError() {
	return ai_nnactor_get_error(m_network);
}

void Gym_LogError(ai_error err, char* title) {
	if (title) {
		printf("AI error (%s) - type=%d code=%d\n", title, err.type, err.code);
	} else {
		printf("AI error - type=%d code=%d\n", err.type, err.code);
	}
}

ai_error Gym_Infer(const Gym_Observation* observation, Gym_Action* action) {
	m_input_data[0] = observation->dist_to_obstacle;
	m_input_data[1] = observation->servo_angle;

	ai_buffer ai_input;
	ai_buffer ai_output;

	ai_input.n_batches = SINGLE_PASS_BATCH_SIZE;
	ai_input.data = AI_HANDLE_PTR(m_input_data);
	ai_output.n_batches = SINGLE_PASS_BATCH_SIZE;
	ai_output.data = AI_HANDLE_PTR(m_output_data);

	ai_nnactor_run(m_network, &ai_input, &ai_output);
	ai_error err = ai_nnactor_get_error(m_network);
	if (err.type != AI_ERROR_NONE) {
		Gym_LogError(err, "ai_nnactor_run");
		return err;
	}

	float* output_buffer = (float*) ai_output.data;
	action->left_wheel_vel = output_buffer[0];
	action->right_wheel_vel = output_buffer[1];

	return err;
}

__STATIC_INLINE const char* BufferFormatToStr(uint32_t val) {
	switch (val) {
	case AI_BUFFER_FORMAT_NONE:
		return "AI_BUFFER_FORMAT_NONE";
	case AI_BUFFER_FORMAT_FLOAT:
		return "AI_BUFFER_FORMAT_FLOAT";
	case AI_BUFFER_FORMAT_U8:
		return "AI_BUFFER_FORMAT_U8";
	case AI_BUFFER_FORMAT_Q15:
		return "AI_BUFFER_FORMAT_Q15";
	case AI_BUFFER_FORMAT_Q7:
		return "AI_BUFFER_FORMAT_Q7";
	default:
		return "UNKNOWN";
	}
}

static ai_u32 GetBufferSize(const ai_buffer* buffer) {
	return buffer->height * buffer->width * buffer->channels;
}

static void PrintLayoutBuffer(const char *msg, const ai_buffer* buffer) {
	printf("%s HWC layout: %d,%d,%ld (s:%ld f:%s)\n", msg, buffer->height,
			buffer->width, buffer->channels, GetBufferSize(buffer),
			BufferFormatToStr(buffer->format));
}
