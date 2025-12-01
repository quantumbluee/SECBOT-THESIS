#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>
#include "imu.h"
#include "gps.h"

typedef struct {
	uint32_t timestamp_ms;

	//IMU
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;

	//GPS
	float latitude;
	float longitude;
	float altitude;

	//OpenMV
	uint16_t img_size;
	uint8_t img_data[256];
}s_node_payload_t;

/**
 * @brief Fill payload with dummy values (for testing without sensors)
 */
void payload_fill_dummy(s_node_payload_t*p);

/**
 * @brief Fill payload from real sensor readings (GPS + IMU)
 */
void payload_fill_from_sensors(s_node_payload_t *p, const gps_fix_t *gps, const imu_data_t *imu);

#endif //PAYLOAD_H
