#ifndef GPS_H
#define GPS_H

#include "usart.h"
#include <stdint.h>

typedef struct {
	float latitude_deg;
	float longitude_deg;
	float altitude_m;
	uint8_t fix_valid;
	uint8_t num_sats;
} gps_fix_t;

/**
 * @brief Initialize GPA driver with given UART handle (eg &huart1)
 *
 */
void GPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief Read and parse a GPS fix from PA1010D using NMEA GGA.
 *
 * This function blocks while reading lines over UART until:
 * - a valid GGA sentence is parsed, or
 * - the timeout (ms) expires.
 *
 * @param fix      Pointer to gps_fix_t to fill.
 * @param timeout_ms   Overall timeout in milliseconds.
 * @return 0 on success
 * 		  -1 on timeout or UART error
 * 		  -3 if only invalid data was received
 */

int GPS_ReadFix(gps_fix_t *fix, uint32_t timeout_ms);

#endif // GPS_H
