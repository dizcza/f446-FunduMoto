#ifndef RINGBUFFER_DMA_H_INCLUDED
#define RINGBUFFER_DMA_H_INCLUDED

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    uint8_t * data;
    uint32_t size;
    DMA_HandleTypeDef * hdma;
    uint8_t const * tail_ptr;
} RingBuffer_DMA;

void RingBuffer_DMA_Init(RingBuffer_DMA * buffer, DMA_HandleTypeDef * hdma, uint8_t * data, uint32_t size);
uint8_t RingBuffer_DMA_GetByte(RingBuffer_DMA * buffer);
uint32_t RingBuffer_DMA_Count(RingBuffer_DMA * buffer);

#endif /* RINGBUFFER_H_INCLUDED */
