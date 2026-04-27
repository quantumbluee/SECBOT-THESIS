/*
 * anode_control.c
 *
 *  Created on: Apr 20, 2026
 *      Author: urvih
 */


#include "anode_control.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static int clamp_pwm(int x)
{
    if (x > 255) return 255;
    if (x < -255) return -255;
    return x;
}

int anode_read_line(UART_HandleTypeDef *huart, char *buf, uint16_t buf_size, uint32_t timeout_ms)
{
    if (!huart || !buf || buf_size < 2) return -1;

    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t c = 0;
        if (HAL_UART_Receive(huart, &c, 1, 10) != HAL_OK) {
            continue;
        }

        if (c == '\r') continue;

        if (c == '\n') {
            if (idx == 0) continue;
            buf[idx] = '\0';
            return 0;
        }

        if (idx < (buf_size - 1)) {
            buf[idx++] = (char)c;
        }
    }

    return -1;
}

int anode_parse_cmd(const char *line, motor_cmd_t *cmd)
{
    if (!line || !cmd) return -1;

    char tag[8] = {0};
    int seq = 0;
    int left = 0;
    int right = 0;

    int n = sscanf(line, "%7[^,],%d,%d,%d", tag, &seq, &left, &right);
    if (n != 4) return -1;
    if (strcmp(tag, "CMD") != 0) return -1;

    cmd->seq = (uint32_t)seq;
    cmd->left_speed = (int16_t)clamp_pwm(left);
    cmd->right_speed = (int16_t)clamp_pwm(right);
    cmd->valid = 1;

    return 0;
}

/* Replace these with your actual CubeMX pins/timer channels */

void motor_left_set(int speed)
{
    int pwm = abs(clamp_pwm(speed));

    if (speed > 0) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htimX, TIM_CHANNEL_Y, pwm);  // replace htimX/channel
}

void motor_right_set(int speed)
{
    int pwm = abs(clamp_pwm(speed));

    if (speed > 0) {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htimZ, TIM_CHANNEL_W, pwm);  // replace htimZ/channel
}

void motors_apply(const motor_cmd_t *cmd)
{
    if (!cmd || !cmd->valid) return;
    motor_left_set(cmd->left_speed);
    motor_right_set(cmd->right_speed);
}
