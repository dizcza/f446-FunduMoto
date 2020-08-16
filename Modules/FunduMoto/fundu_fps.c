/*
 * fundu_fps.c
 *
 *  Created on: Aug 11, 2020
 *      Author: dizcza
 */

#include <math.h>
#include "stm32f4xx_hal.h"
#include "fundumoto.h"

static float m_fps = 0.f;
static uint32_t m_last_tick = 0U;


float FunduMoto_GetFPS() {
	uint32_t tick = HAL_GetTick();
	if (m_last_tick != 0U) {
		float period = (float) tick - m_last_tick;
		period = fmaxf(period, 1.f);
		m_fps = 1000.0f / period;  // ticks are in ms
	}
	m_last_tick = tick;
	return m_fps;
}
