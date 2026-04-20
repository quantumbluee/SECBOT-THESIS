/*
 * vision.c
 *
 *  Created on: Apr 19, 2026
 *      Author: urvih
 */

#include "vision.h"
#include <string.h>
#include <stdio.h>

int vision_read_line(UART_HandleTypeDef *huart,
		char *buf,
		uint16_t buf_size,
		uint32_t timeout_ms)
{
	if (!huart || !buf || buf_size < 2) return -1;

	uint16_t idx = 0;
	uint32_t start = HAL_GetTick();

	while ((HAL_GetTick()- start) < timeout_ms){
		uint8_t c;

		if (HAL_UART_Receive(huart, &c, 1, 10) != HAL_OK)
			continue;

		if (c=='\r') continue;

		if (c=='\n'){
			if (idx == 0) continue;
			buf[idx] = '\0';
			return 0;
		}

		if (idx < buf_size - 1){
			buf[idx++] = (char)c;
		}
	}
	return -1;
}


int vision_parse_line(const char *line, vision_data_t *vision){
	if (!line || !vision) return -1;

	const char *start = strstr(line, "VISION,");
	if (!start) return -1;

	char tag[8] = {0};
	int line_found = 0;
	int line_error = 0;
	int obstacle_flag = 0;
	int confidence = 0;

	int n = sscanf(line, "%7[^,],%d,%d,%d,%d",
			tag, &line_found, &line_error,
			&obstacle_flag, &confidence);

	if (n!=5) return -1;

	vision -> valid = (line_found || obstacle_flag) ? 1:0;
	vision -> line_error = (int16_t)line_error;
	vision -> obstacle_flag = (uint8_t)obstacle_flag;
	vision -> confidence = (uint8_t)confidence;

	return 0;
}
