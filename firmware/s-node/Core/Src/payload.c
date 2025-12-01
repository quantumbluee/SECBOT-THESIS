#include "payload.h"
#include "gps.h"
#include "imu.h"
#include "main.h"
#include <string.h>

void payload_fill_dummy(s_node_payload_t*p)
{
	if(!p) return;

	p->timestamp_ms += 100; //fake time step

	//FAKE IMU
	p->accel_x=0.1f;
	p->accel_y=0.2f;
	p->accel_z=0.3f;
	p->gyro_x=1.1f;
	p->gyro_y=1.2f;
	p->gyro_z=1.3f;

	//FAKE GPS
	p->latitude=40.0f;
	p->longitude=-75.0f;
	p->altitude=12.3f;

	//FAKE IMAGE BYTES
	p->img_size=10;
	for (uint16_t i=0; i<p->img_size;i++){
		p->img_data[i] = (uint8_t)i;
	}
}

void payload_fill_from_sensors(s_node_payload_t*p, const gps_fix_t *gps, const imu_data_t *imu)
{
	if(!p) return;
	//Allow gps/imu to be NULL in case something failed upstream
	memset(p, 0, sizeof(*p));

	p->timestamp_ms=HAL_GetTick();

	//IMU
	if(imu){
	p->accel_x=imu->accel_x;
	p->accel_y=imu->accel_y;
	p->accel_z=imu->accel_z;
	p->gyro_x=imu->gyro_x;
	p->gyro_y=imu->gyro_y;
	p->gyro_z=imu->gyro_z;
	}

	//GPA (only valid if fix)
	if(gps){
	if(gps->fix_valid){
		p->latitude=gps->latitude_deg;
		p->longitude=gps->longitude_deg;
		p->altitude=gps->altitude_m;
	}else{
		p->latitude=p->longitude=p->altitude=0.0f;
	}
	}

	//OperMV placeholder
	p->img_size=0;
}
