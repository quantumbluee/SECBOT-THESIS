#include "frame.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

int frame_build(uint8_t *out,
                uint16_t out_max,
                const uint8_t *payload,
                uint16_t payload_len,
                const uint8_t hash[FRAME_HASH_SIZE])
{
    if (out == NULL || payload == NULL || hash == NULL)
        return -1;

    // Total = start + len(2) + payload + hash(32) + end
    uint16_t total_len = 1 + 2 + payload_len + FRAME_HASH_SIZE + 1;

    if (out_max < total_len)
        return -2;

    out[0] = FRAME_START_BYTE;
    out[1] = (uint8_t)((payload_len >> 8) & 0xFF);
    out[2] = (uint8_t)(payload_len & 0xFF);

    for (uint16_t i = 0; i < payload_len; i++) {
        out[3 + i] = payload[i];
    }

    for (uint16_t i = 0; i < FRAME_HASH_SIZE; i++) {
        out[3 + payload_len + i] = hash[i];
    }

    out[3 + payload_len + FRAME_HASH_SIZE] = FRAME_END_BYTE;

    return total_len;
}
