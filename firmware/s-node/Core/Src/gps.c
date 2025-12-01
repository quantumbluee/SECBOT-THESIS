#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define GPS_LINE_MAX 128

static UART_HandleTypeDef *gps_huart = NULL;

/* Convert NMEA coordinate (ddmm.mmmm or dddmm.mmmm) to signed degrees. */
static float nmea_coord_to_deg(const char *coord, char hemi)
{
    if (coord == NULL || coord[0] == '\0')
        return 0.0f;

    float raw = atof(coord);                 // e.g., 4042.6142
    if (raw == 0.0f)
        return 0.0f;

    float degrees = floorf(raw / 100.0f);    // 40
    float minutes = raw - degrees * 100.0f;  // 42.6142
    float deg = degrees + (minutes / 60.0f); // 40 + 42.6142/60

    if (hemi == 'S' || hemi == 'W')
        deg = -deg;

    return deg;
}

/* Read one NMEA line (ending in '\n') into buf, NUL-terminated. */
static int gps_read_line(char *buf, uint16_t buf_size, uint32_t timeout_ms)
{
    if (gps_huart == NULL || buf == NULL || buf_size < 2)
        return -1;

    uint16_t idx = 0;
    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        uint8_t c;
        if (HAL_UART_Receive(gps_huart, &c, 1, 10) != HAL_OK)
        {
            // check global timeout
            if ((HAL_GetTick() - start_tick) > timeout_ms)
                return -1;
            continue;
        }

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (idx == 0)
                continue; // skip empty lines

            buf[idx] = '\0';
            return 0;     // success
        }

        if (idx < buf_size - 1)
        {
            buf[idx++] = (char)c;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms)
            return -1;
    }
}

/* Parse a GGA or GNGGA sentence and fill gps_fix_t. */
static int gps_parse_gga(char *line, gps_fix_t *fix)
{
    // Accept $GPGGA or $GNGGA
    if (strncmp(line, "$GPGGA", 6) != 0 &&
        strncmp(line, "$GNGGA", 6) != 0)
    {
        return -1;
    }

    // Tokenize
    char *token;
    int field = 0;

    char lat_str[16] = {0};
    char lon_str[16] = {0};
    char lat_hemi = 'N';
    char lon_hemi = 'E';
    int fix_quality = 0;
    int num_sats = 0;
    float altitude = 0.0f;

    token = strtok(line, ",");
    while (token != NULL)
    {
        switch (field)
        {
        case 2: // latitude
            if (token[0] != '\0')
                strncpy(lat_str, token, sizeof(lat_str) - 1);
            break;
        case 3: // N/S
            if (token[0] != '\0')
                lat_hemi = token[0];
            break;
        case 4: // longitude
            if (token[0] != '\0')
                strncpy(lon_str, token, sizeof(lon_str) - 1);
            break;
        case 5: // E/W
            if (token[0] != '\0')
                lon_hemi = token[0];
            break;
        case 6: // fix quality (0=no fix)
            fix_quality = atoi(token);
            break;
        case 7: // number of satellites
            num_sats = atoi(token);
            break;
        case 9: // altitude in meters
            altitude = (float)atof(token);
            break;
        default:
            break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    if (fix == NULL)
        return -1;

    fix->latitude_deg  = nmea_coord_to_deg(lat_str, lat_hemi);
    fix->longitude_deg = nmea_coord_to_deg(lon_str, lon_hemi);
    fix->altitude_m    = altitude;
    fix->num_sats      = (uint8_t)num_sats;
    fix->fix_valid     = (fix_quality > 0) ? 1 : 0;

    return 0;
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    gps_huart = huart;
}

int GPS_ReadFix(gps_fix_t *fix, uint32_t timeout_ms)
{
    if (gps_huart == NULL || fix == NULL)
        return -1;

    uint32_t start = HAL_GetTick();

    char line[GPS_LINE_MAX];

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (gps_read_line(line, sizeof(line), timeout_ms) != 0)
            return -1; // timeout or UART error
        printf("NMEA: %s\r\n", line);

        // Make a working copy since gps_parse_gga uses strtok
        char work[GPS_LINE_MAX];
        strncpy(work, line, sizeof(work) - 1);
        work[sizeof(work) - 1] = '\0';

        if (gps_parse_gga(work, fix) == 0)
        {
            // We got a GGA sentence; return even if fix_valid = 0
            return 0;
        }
    }

    return -2; // no valid GGA in time window
}
