#include "hash.h"
#include "frame.h"
#include <string.h>
#include "main.h"
#include <stddef.h>
#include "mbedtls/md.h"

int HASH_HMAC_SHA256(const uint8_t *key, uint32_t key_len, const uint8_t *data, uint8_t data_len, uint8_t out[HASH_SHA256_SIZE]){
	if(key == NULL || data == NULL || out == NULL){
		return -1;
	}
	const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
	if (md_info == NULL){
		return -2;
	}

	int ret = mbedtls_md_hmac(md_info, key, key_len, data, data_len, out);
	if (ret != 0){
		return -3;
	}

	return 0;

}
