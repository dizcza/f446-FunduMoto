/*
 * gym.c
 *
 *  Created on: Oct 20, 2019
 *      Author: Danylo Ulianych, d.ulianych@gmail.com
 */


#include <nnactor.h>
#include <nnactor_data.h>
#include <stdio.h>
#include <sys/_stdint.h>

#include "stm32f4xx_hal.h"
#include "gym.h"

#define SINGLE_PASS_BATCH_SIZE 1

static ai_float m_input_data[AI_NNACTOR_IN_1_SIZE];
static ai_float m_output_data[AI_NNACTOR_OUT_1_SIZE];

static ai_handle m_network = AI_HANDLE_NULL;

static ai_buffer ai_input[AI_NNACTOR_IN_NUM] = AI_NNACTOR_IN;
static ai_buffer ai_output[AI_NNACTOR_OUT_NUM] = AI_NNACTOR_OUT;

/* Global buffer to handle the activations data buffer - R/W data */
AI_ALIGNED(4)
static ai_u8 m_activations[AI_NNACTOR_DATA_ACTIVATIONS_SIZE];


static void assert_in_valid_range(float val) {
	// NN requires input to be in [0, 1] range.
	assert_param(val >= 0.f && val <= 1.f);
}


ai_error Gym_InitNetwork() {
	ai_error err;

    const ai_network_params params = {
            AI_NNACTOR_DATA_WEIGHTS(ai_nnactor_data_weights_get()),
            AI_NNACTOR_DATA_ACTIVATIONS(m_activations)
    };

	err = ai_nnactor_create(&m_network, AI_NNACTOR_DATA_CONFIG);
	if (err.type != AI_ERROR_NONE) {
		Gym_LogError(err, "ai_nnactor_create");
		ai_nnactor_destroy(m_network);
		return err;
	}

	if (!ai_nnactor_init(m_network, &params)) {
		err = ai_nnactor_get_error(m_network);
		Gym_LogError(err, "ai_nnactor_init");
		ai_nnactor_destroy(m_network);
		m_network = AI_HANDLE_PTR(NULL);
	}

	return err;
}

ai_error Gym_GetError() {
	return ai_nnactor_get_error(m_network);
}

ai_handle Gym_NetworkHandle() {
	return m_network;
}

void Gym_LogError(ai_error err, char* title) {
	if (title) {
		printf("AI error (%s) - type=%d code=%d\n", title, err.type, err.code);
	} else {
		printf("AI error - type=%d code=%d\n", err.type, err.code);
	}
}

ai_error Gym_Infer(const Gym_Observation* observation, Gym_Action* action) {
	assert_in_valid_range(observation->dist_to_obstacle);
	assert_in_valid_range(observation->servo_angle);

	m_input_data[0] = observation->dist_to_obstacle;
	m_input_data[1] = observation->servo_angle;

	ai_input[0].n_batches = SINGLE_PASS_BATCH_SIZE;
	ai_input[0].data = AI_HANDLE_PTR((const void*)m_input_data);
	ai_output[0].n_batches = SINGLE_PASS_BATCH_SIZE;
	ai_output[0].data = AI_HANDLE_PTR((void*)m_output_data);

	ai_nnactor_run(m_network, &ai_input[0], &ai_output[0]);
	ai_error err = ai_nnactor_get_error(m_network);
	if (err.type != AI_ERROR_NONE) {
		Gym_LogError(err, "ai_nnactor_run");
		return err;
	}

	float* output_buffer = (float*) ai_output[0].data;
	action->left_wheel_vel = output_buffer[0];
	action->right_wheel_vel = output_buffer[1];

	return err;
}

static const char* BufferFormatToStr(uint32_t val) {
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

void Gym_LogNetworkInfo() {
	ai_network_report report;
	if (ai_nnactor_get_info(m_network, &report)) {
		printf(">>> Network configuration\n");
		printf(" Model name: %s\n", report.model_name);
		printf(" Model datetime: %s\n", report.model_datetime);
		printf(">>> Network info\n");
		printf("  nodes: %ld\n", report.n_nodes);
		printf("  complexity: %ld MACC\n", report.n_macc);
		printf("  activation: %ld bytes\n", GetBufferSize(&report.activations));
		printf("  params: %ld bytes\n", GetBufferSize(&report.params));
		printf("  inputs/outputs: %u/%u\n", report.n_inputs, report.n_outputs);
		PrintLayoutBuffer("  IN tensor format:", report.inputs);
		PrintLayoutBuffer("  OUT tensor format:", report.outputs);
	} else {
		ai_error err = ai_nnactor_get_error(m_network);
		Gym_LogError(err, "ai_grunet_get_info");
	}
}

