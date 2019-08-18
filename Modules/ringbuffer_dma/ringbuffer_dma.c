#include "ringbuffer_dma.h"

uint32_t RingBuffer_DMA_Count(RingBuffer_DMA * buffer) {
	// get counter returns the number of remaining data units in the current DMA Stream transfer (total size - received count)
	// current head = start + (size - received count)
	uint8_t const * head = buffer->data + buffer->size - __HAL_DMA_GET_COUNTER(buffer->hdma);
	uint8_t const * tail = buffer->tail_ptr;
	if (head >= tail)
		return head - tail;
	else
		return head - tail + buffer->size;
}

uint8_t RingBuffer_DMA_GetByte(RingBuffer_DMA * buffer) {
	// get counter returns the number of remaining data units in the current DMA Stream transfer (total size - received count)
	// current head = start + (size - received count)
	uint8_t const * head = buffer->data + buffer->size - __HAL_DMA_GET_COUNTER(buffer->hdma);
	uint8_t const * tail = buffer->tail_ptr;

	if (head != tail) {
		uint8_t c = *buffer->tail_ptr++;
		if (buffer->tail_ptr >= buffer->data + buffer->size) {
			buffer->tail_ptr -= buffer->size;
		}
		return c;
	}

	return 0;
}

void RingBuffer_DMA_Init(RingBuffer_DMA * buffer, DMA_HandleTypeDef * hdma, uint8_t * data, uint32_t size){
	buffer->data = data; // set array
	buffer->size = size; // and its size
	buffer->hdma = hdma; // initialized DMA
	buffer->tail_ptr = data; // tail == head == start of array
}
