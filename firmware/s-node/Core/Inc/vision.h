/*
 * vision.h
 *
 *  Created on: Apr 19, 2026
 *      Author: urvih
 */

#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "payload.h"
#include <stdint.h>


// Read one line from OpenMV UART
int vision_read_line(UART_HandleTypeDef *huart,
		char *buf,
		uint16_t buf_size,
		uint32_t timeout_ms);

// Parse "VISION,..." string into struct
int vision_parse_line(const char *line, vision_data_t *vision);

#endif


