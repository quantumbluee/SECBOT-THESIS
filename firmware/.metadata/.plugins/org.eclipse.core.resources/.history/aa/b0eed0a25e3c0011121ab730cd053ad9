#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>
#include "imu.h"
#include "gps.h"

#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp_ms;

    // GPS
    uint8_t  gps_valid;
    int32_t  latitude_e7;
    int32_t  longitude_e7;
    int32_t  altitude_mm;
    uint8_t  num_sats;

    // IMU
    uint8_t  imu_valid;
    int16_t  accel_x_mg;
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int16_t  gyro_x_centi_dps;
    int16_t  gyro_y_centi_dps;
    int16_t  gyro_z_centi_dps;
    int16_t  temp_centi_c;

    // OpenMV summary
    uint8_t  vision_valid;
    int16_t  line_error;
    uint8_t  obstacle_flag;
    uint8_t  vision_confidence;
} s_node_payload_t;
#pragma pack(pop)

void payload_fill_dummy(s_node_payload_t *p);
void payload_fill_from_sensors(s_node_payload_t *p, const gps_fix_t *gps, const imu_data_t *imu);

#endif
