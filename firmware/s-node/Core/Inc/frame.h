#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

#define FRAME_START_BYTE   0x7E
#define FRAME_END_BYTE     0x7F
#define FRAME_HASH_SIZE    32   // 32-byte SHA-256 hash

/**
 * @brief Build a frame from payload + hash.
 *
 * Frame format:
 * [START][LEN_H][LEN_L][PAYLOAD...][HASH(32B)][END]
 *
 * LEN = payload length ONLY (big-endian)
 *
 * @param out        Output buffer for frame
 * @param out_max    Size of output buffer
 * @param payload    Pointer to payload bytes
 * @param payload_len Length of payload in bytes
 * @param hash       32-byte SHA-256 hash over payload
 *
 * @return >=0: frame length in bytes
 *         <0 : error (buffer too small, bad args)
 */
int frame_build(uint8_t *out,
                uint16_t out_max,
                const uint8_t *payload,
                uint16_t payload_len,
                const uint8_t hash[FRAME_HASH_SIZE]);

#endif // FRAME_H
