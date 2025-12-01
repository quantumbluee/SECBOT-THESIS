#include "frame.h"

/*
 * frame format: [START][LEN_H][PAYLOAD....][HASH(32B)][END][END]
 * LEN = payload_len + FRAME_HASH_SIZE
 */
int frame_build(uint8_t *out,
                uint16_t out_max,
                const uint8_t *payload,
                uint16_t payload_len,
                const uint8_t hash[FRAME_HASH_SIZE])
{
    if (!out || !payload || !hash){
    	return -1;
    }

    // Required frame size:
    // Total length inside frame
    uint16_t inner_len = payload_len + FRAME_HASH_SIZE;
    //we need 1(start) + 2(len bytes) + inner_len + 1(end)
    uint32_t needed = 1u + 2u + inner_len + 1u;
    if (needed > out_max){
    	return -2;
    }   // output buffer too small

    uint16_t idx = 0;

    // START BYTE
    out[idx++] = FRAME_START_BYTE;

    // LENGTH BYTES (big endian)
    out[idx++] = (uint8_t)((inner_len >> 8) & 0xFF);
    out[idx++] = (uint8_t)(inner_len & 0xFF);

    // PAYLOAD BYTES
    for (uint16_t i = 0; i < payload_len; i++){
    	out[idx++] = payload[i];
    }


    // HASH BYTES (32 bytes SHA-256)
    for (uint16_t i = 0; i < FRAME_HASH_SIZE; i++){
    	out[idx++] = hash[i];
    }


    // END BYTE
    out[idx++] = FRAME_END_BYTE;

    return (int)idx;   // final frame length
}
