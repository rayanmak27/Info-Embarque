#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <stdbool.h>

#include "../Raspi/raspi.h"

#define UART_ERR_BAUDRATE_INVALID -1
#define UART_ERR_RASPI_INVALID -2
#define UART_ERR_IMPOSSIBLE_TO_READ_UART -3
#define UART_ERR_IMPOSSIBLE_TO_INIT_STRUCT -4
#define UART_ERR_IMPOSSIBLE_TO_PARAM_UART -5

int32_t UART_Init(RASPI raspi, uint32_t baud);
void UART_Close(int32_t uart);

static bool UART_IsValidBaudRate(uint32_t baud);

#endif