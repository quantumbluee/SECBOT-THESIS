#include "usart.h"
#include <stdio.h>


int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
