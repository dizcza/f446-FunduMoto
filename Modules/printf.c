/*
 * printf.c
 *
 *  Created on: Aug 10, 2020
 *      Author: dizcza
 */


#include <stdio.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"

#define UartHandle huart4

extern UART_HandleTypeDef huart4;


#if defined(__GNUC__)

#include <errno.h>
#include <sys/unistd.h> /* STDOUT_FILENO, STDERR_FILENO */

int _write(int fd, const void *buff, int count)
{
    HAL_StatusTypeDef status;

    if ((count < 0) && (fd != STDOUT_FILENO) && (fd != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

    status = HAL_UART_Transmit(&UartHandle, (uint8_t *)buff, count,
            HAL_MAX_DELAY);

    return (status == HAL_OK ? count : 0);
}

#endif  // __GNUC__
