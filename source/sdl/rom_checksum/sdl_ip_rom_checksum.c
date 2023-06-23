/*********************************************************************
 *   Copyright (c) Texas Instruments Incorporated 2023
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file     sdl_ip_rom_checksum.c
 *
 *  \brief    This file contains the implementation of the low level API's present in the ROM Checksum diagnostics.
 */

#include <stdint.h>
#include "sdl_ip_rom_checksum.h"
#include <stdbool.h>
#include <sdl/include/sdl_types.h>

#define SDL_STORE64H(x, y) \
 	uint8_t *y8 = (uint8_t *)y; \
  	y8[0] = ((x>>56) & (uint8_t)255); \
  	y8[1] = ((x>>48) & (uint8_t)255); \
	y8[2] = ((x>>40) & (uint8_t)255); \
	y8[3] = ((x>>32) & (uint8_t)255); \
	y8[4] = ((x>>24) & (uint8_t)255); \
	y8[5] = ((x>>16) & (uint8_t)255); \
	y8[6] = ((x>>8) & (uint8_t)255); \
	y8[7] = ((x>>0) & (uint8_t)255);

#define SDL_LOAD64H(x, y)                                                      \
		x = (((uint64_t)((y)[0] & (uint64_t)255))<<56)|(((uint64_t)((y)[1] & (uint64_t)255))<<48) | \
		(((uint64_t)((y)[2] & (uint64_t)255))<<40)|(((uint64_t)((y)[3] & (uint64_t)255))<<32) | \
		(((uint64_t)((y)[4] & (uint64_t)255))<<24)|(((uint64_t)((y)[5] & (uint64_t)255))<<16) | \
		(((uint64_t)((y)[6] & (uint64_t)255))<<8)|(((uint64_t)((y)[7] & (uint64_t)255)));

#define RND(a,b,c,d,e,f,g,h,i)                    \
	t0 = h + SDL_Sigma1((uint64_t)e) + SDL_ch(e, f, g) + K[i] + W[i];   \
	t1 = SDL_Sigma0((uint64_t)a) + SDL_Maj(a, b, c);                  \
	d += t0;                                        \
	h  = t0 + t1;
/* Right rotate circular shift by y bits. */
#define SDL_ROR64c(x, y) \
	( ((((uint64_t)(x)&SDL_CONST64(0xFFFFFFFFFFFFFFFFU))>>((uint64_t)(y)&SDL_CONST64(63))) | \
	   ((uint64_t)(x)<<((uint64_t)((uint64_t)64-((uint64_t)(y)&SDL_CONST64(63)))))) & SDL_CONST64(0xFFFFFFFFFFFFFFFFU))

/* the K array */ //These are round constant from 0-79
static const uint64_t K[80] = {
    0x428A2F98D728AE22U, 0x7137449123EF65CDU, 0xB5C0FBCFEC4D3B2FU, 0xE9B5DBA58189DBBCU,
    0x3956C25BF348B538U, 0x59F111F1B605D019U, 0x923F82A4AF194F9BU, 0xAB1C5ED5DA6D8118U,
    0xD807AA98A3030242U, 0x12835B0145706FBEU, 0x243185BE4EE4B28CU, 0x550C7DC3D5FFB4E2U,
    0x72BE5D74F27B896FU, 0x80DEB1FE3B1696B1U, 0x9BDC06A725C71235U, 0xC19BF174CF692694U,
    0xE49B69C19EF14AD2U, 0xEFBE4786384F25E3U, 0x0FC19DC68B8CD5B5U, 0x240CA1CC77AC9C65U,
    0x2DE92C6F592B0275U, 0x4A7484AA6EA6E483U, 0x5CB0A9DCBD41FBD4U, 0x76F988DA831153B5U,
    0x983E5152EE66DFABU, 0xA831C66D2DB43210U, 0xB00327C898FB213FU, 0xBF597FC7BEEF0EE4U,
    0xC6E00BF33DA88FC2U, 0xD5A79147930AA725U, 0x06CA6351E003826FU, 0x142929670A0E6E70U,
    0x27B70A8546D22FFCU, 0x2E1B21385C26C926U, 0x4D2C6DFC5AC42AEDU, 0x53380D139D95B3DFU,
    0x650A73548BAF63DEU, 0x766A0ABB3C77B2A8U, 0x81C2C92E47EDAEE6U, 0x92722C851482353BU,
    0xA2BFE8A14CF10364U, 0xA81A664BBC423001U, 0xC24B8B70D0F89791U, 0xC76C51A30654BE30U,
    0xD192E819D6EF5218U, 0xD69906245565A910U, 0xF40E35855771202AU, 0x106AA07032BBD1B8U,
    0x19A4C116B8D2D0C8U, 0x1E376C085141AB53U, 0x2748774CDF8EEB99U, 0x34B0BCB5E19B48A8U,
    0x391C0CB3C5C95A63U, 0x4ED8AA4AE3418ACBU, 0x5B9CCA4F7763E373U, 0x682E6FF3D6B2B8A3U,
    0x748F82EE5DEFB2FCU, 0x78A5636F43172F60U, 0x84C87814A1F0AB72U, 0x8CC702081A6439ECU,
    0x90BEFFFA23631E28U, 0xA4506CEBDE82BDE9U, 0xBEF9A3F7B2C67915U, 0xC67178F2E372532BU,
    0xCA273ECEEA26619CU, 0xD186B8C721C0C207U, 0xEADA7DD6CDE0EB1EU, 0xF57D4F7FEE6ED178U,
    0x06F067AA72176FBAU, 0x0A637DC5A2C898A6U, 0x113F9804BEF90DAEU, 0x1B710B35131C471BU,
    0x28DB77F523047D84U, 0x32CAAB7B40C72493U, 0x3C9EBE0A15C9BEBCU, 0x431D67C49C100D4CU,
    0x4CC5D4BECB3E42B6U, 0x597F299CFC657E2AU, 0x5FCB6FAB3AD6FAECU, 0x6C44198C4A475817U
 };

void SDL_memcpy(void *dest, void *src, uint32_t n){
	/* Copy contents of src[] to dest[] */
	unsigned char *cdest = (unsigned char *)dest;
	unsigned char *csrc  = (unsigned char *)src;
	for (uint32_t i=0; i < n; i++){
    	cdest[i] = csrc[i];
    }
}
/* compress 1024-bits */
/* md -> state is 8 byte seed value.
buf -> 16 word data each of 64 bits.
output 8 byte seed value for next round.
*/
/**
 *  Design: PROC_SDL-6265
 */

void  SDL_ROM_Checksum_compress(SDL_ROM_Checksum_obj * md, uint8_t *buf)
{
	uint64_t S[8], W[80], t0, t1;
	int32_t i = 0;

	/* copy state into S */
	for (i = 0; i < 8; i++) {
		S[i] = md->state[i];
	}

	/* copy the state into 1024-bits into W[0..15] */
	for (i = 0; i < 16; i++) {
		SDL_LOAD64H(W[i], buf + (8*i));
	}

	/* fill W[16..79] */
	for (i = 16; i < 80; i++) {
		W[i] = SDL_Gamma1(W[i - 2]) + W[i - 7] + SDL_Gamma0(W[i - 15]) + W[i - 16];
	}
	for (i = 0; i < 80; i += 8) {
		RND(S[0],S[1],S[2],S[3],S[4],S[5],S[6],S[7],i+0);
		RND(S[7],S[0],S[1],S[2],S[3],S[4],S[5],S[6],i+1);
		RND(S[6],S[7],S[0],S[1],S[2],S[3],S[4],S[5],i+2);
		RND(S[5],S[6],S[7],S[0],S[1],S[2],S[3],S[4],i+3);
		RND(S[4],S[5],S[6],S[7],S[0],S[1],S[2],S[3],i+4);
		RND(S[3],S[4],S[5],S[6],S[7],S[0],S[1],S[2],i+5);
		RND(S[2],S[3],S[4],S[5],S[6],S[7],S[0],S[1],i+6);
		RND(S[1],S[2],S[3],S[4],S[5],S[6],S[7],S[0],i+7);
	}
	/* feedback */
	for (i = 0; i < 8; i++) {
		md->state[i] = md->state[i] + S[i];
	}
}

/**
  Process a block of memory though the hash
  @param md     The hash state
  @param in     The data to hash
  @param inlen  The length of the data (octets)
  @return SDL_PASS if successful
 */

/**
 *  Design: PROC_SDL-6263
 */
int32_t SDL_ROM_Checksum_process (SDL_ROM_Checksum_obj * md, uint8_t *in, uint32_t inlen)
{
	uint32_t n;
	int32_t sdl_result= SDL_PASS;
	uint8_t *in1 = in;
	uint32_t inlen1 = inlen;
	if ((md->curlen) >= ((uint32_t)sizeof(md->buf))) {
		sdl_result = SDL_EFAIL;
	}


	if(sdl_result == SDL_PASS){
		while (inlen1 > (uint32_t)0) {
			if (inlen1 >= (uint32_t)SDL_BLOCK_SIZE) {
				SDL_ROM_Checksum_compress (md, (uint8_t *)in1);
					md->length += (SDL_BLOCK_SIZE) * (uint8_t)8;
					in1             += 128;
					inlen1          -= (uint32_t)SDL_BLOCK_SIZE;
			} else {
					n = SDL_MIN(inlen1, (SDL_BLOCK_SIZE - md->curlen));
					SDL_memcpy(md->buf + (md->curlen), (void *)in1, n);
					md->curlen += n;
					in1             += (int)n;
					inlen1          -= n;
				}
		}
	}
	return sdl_result;
}

/**
  Initialize the hash state
  @param md   The hash state you wish to initialize
 */

/**
 *  Design: PROC_SDL-6262
 */
void SDL_ROM_Checksum_init(SDL_ROM_Checksum_obj * const md)
{
	md->curlen = 0;
	md->length = 0;
	md->state[0] = SDL_CONST64(0x6a09e667f3bcc908U);
	md->state[1] = SDL_CONST64(0xbb67ae8584caa73bU);
	md->state[2] = SDL_CONST64(0x3c6ef372fe94f82bU);
	md->state[3] = SDL_CONST64(0xa54ff53a5f1d36f1U);
	md->state[4] = SDL_CONST64(0x510e527fade682d1U);
	md->state[5] = SDL_CONST64(0x9b05688c2b3e6c1fU);
	md->state[6] = SDL_CONST64(0x1f83d9abfb41bd6bU);
	md->state[7] = SDL_CONST64(0x5be0cd19137e2179U);
}

/**
  Terminate the hash to get the digest
  @param md  The hash state
  @param out [out] The destination of the hash (64 bytes)
  @return SDL_PASS if successful
 */
/**
 *  Design: PROC_SDL-6266
 */
int32_t SDL_ROM_Checksum_done(SDL_ROM_Checksum_obj * md)
{
	int32_t test_result = SDL_PASS;

	if ((md->curlen) >= ((uint32_t)sizeof(md->buf))) {
			test_result = SDL_EFAIL;
	}
	if(test_result == SDL_PASS){
		/* increase the length of the message */
		md->length += md->curlen * SDL_CONST64(8);

		/* append the '1' bit */
		md->buf[md->curlen] = 0x80U;
		md->curlen = md->curlen+(uint32_t)1;

		/* pad upto 120 bytes of zeroes
		* note: that from 112 to 120 is the 64 MSB of the length.  We assume that you won't hash
		* > 2^64 bits of data... :-)
		*/
		while ((md->curlen) < ((uint32_t)120U)) {
			md->buf[md->curlen] = 0;
			md->curlen ++;
		}

		/* store length */
		SDL_STORE64H(md->length, md->buf+(uint8_t)120U);
		SDL_ROM_Checksum_compress(md, md->buf);
	}
	return test_result;

}

/*
 Compare resultant buffer with original golden vector value.
*/
/**
 *  Design: PROC_SDL-6264
 */

int32_t SDL_ROM_Checksum_compareResult (uint64_t buflen, SDL_ROM_Checksum_obj * md,  uint64_t * golden_value){
	uint64_t test = 0;
	int32_t sdl_result = SDL_PASS;
	uint64_t * golden_value1 = golden_value;
	for(uint64_t i=0; i<buflen; i++){
		if(sdl_result == SDL_PASS){
			test = *(golden_value1);
			test = ((0xFF00000000000000U & test)>>56) | ((0x00FF000000000000U & test)>>40) |
			((0x0000FF0000000000U & test)>>24) |	((0x000000FF00000000U & test)>>8) |
			((0x00000000000000FFU & test)<<56) | ((0x000000000000FF00U & test)<<40) |
			((0x0000000000FF0000U & test)<<24) |	((0x00000000FF000000U & test)<<8);
			if(md->state[i]== test){
				golden_value1++;
			}
			else{
				sdl_result = SDL_EFAIL;
			}
		}
	}
	return sdl_result;
 }
