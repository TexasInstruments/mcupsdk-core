#ifndef SHA512_H
#define SHA512_H

/*************************** HEADER FILES ***************************/
#include <stddef.h>
#include <stdint.h>

/****************************** MACROS ******************************/
#define SHA512_BLOCK_LENGTH 128
#define SHA512_DIGEST_LENGTH 64

// SHA-512 context structure
typedef struct _SHA512_CTX {
	uint64_t	state[8];
	uint64_t	bitcount[2];
	uint8_t	buffer[SHA512_BLOCK_LENGTH];
} SHA512_CTX;

/*********************** FUNCTION DECLARATIONS **********************/
void sha512_init(SHA512_CTX* context);
void sha512_update(SHA512_CTX* context, void *datain, size_t len);
void sha512_final(SHA512_CTX* context, uint8_t digest[]);

#endif   // SHA512_H