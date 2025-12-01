#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "stm32f7xx_hal.h"   // change to your HAL header if needed

// -------- Public types --------

typedef struct {
    float accel_x;   // g
    float accel_y;
    float accel_z;
    float gyro_x;    // deg/s
    float gyro_y;
    float gyro_z;
    float temp_c;    // optional
} imu_data_t;

// -------- API --------

// Call once at boot
int IMU_Init(I2C_HandleTypeDef *hi2c);

// Read accel+gyro+temp (polling)
int IMU_Read(imu_data_t *out);

// Optional helpers
int IMU_WhoAmI(uint8_t *whoami);

#endif // IMU_H
