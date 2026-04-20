#ifndef ANODE_CONTROL_H
#define ANODE_CONTROL_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint32_t seq;
    int16_t left_speed;
    int16_t right_speed;
    uint8_t valid;
} motor_cmd_t;

int anode_read_line(UART_HandleTypeDef *huart, char *buf, uint16_t buf_size, uint32_t timeout_ms);
int anode_parse_cmd(const char *line, motor_cmd_t *cmd);

void motor_left_set(int speed);
void motor_right_set(int speed);
void motors_apply(const motor_cmd_t *cmd);

#endif
