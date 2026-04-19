#ifndef HASH_H
#define HASH_H

#include <stdint.h>

#define HASH_SHA256_SIZE 32

/**
 * @brief Compute SHA-256 over input data
 *
 * @param data Input bytes
 * @param len Length in bytes
 * @param out32 32-byte output buffer
 */

int HASH_HMAC_SHA256(const uint8_t *key, uint32_t key_len, const uint8_t *data, uint8_t data_len, uint8_t out[HASH_SHA256_SIZE]);

#endif //HASH_H
