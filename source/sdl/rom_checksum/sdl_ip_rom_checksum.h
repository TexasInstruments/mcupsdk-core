/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *
 *   @file  sdl_ip_rom_checksum.h
 *
 *   @brief This file contains the SDL-FL API's for ROM Checksum
 *
 *
 *
 */
#ifndef _sdl_ip_rom_checksum_H
#define _sdl_ip_rom_checksum_H
#include <inttypes.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Various logical functions */
#define SDL_ch(x,y,z)       (z ^ (x & (y ^ z)))  //this function will (x AND y) XOR (NOT x AND z)
#define SDL_Maj(x,y,z)      (((x | y) & z) | (x & y))
#define SDL_S(x, n)         (SDL_ROR64c(x, n)) //Rotate right circular shift  by n bits
#define SDL_R(x, n)         (((uint64_t)(x)&SDL_CONST64(0xFFFFFFFFFFFFFFFFU))>>((uint64_t)n))  // Rotate left circular shift  by n bits
#define SDL_Sigma0(x)       (SDL_S(x, 28) ^ SDL_S(x, 34) ^ SDL_S(x, 39))
#define SDL_Sigma1(x)       (SDL_S(x, 14) ^ SDL_S(x, 18) ^ SDL_S(x, 41))
#define SDL_Gamma0(x)       (uint64_t)(SDL_S(x, 1) ^ SDL_S(x, 8) ^ SDL_R(x, 7)) // we perform the left and right shift after that we will perform the XOR of all the results
#define SDL_Gamma1(x)       (uint64_t)(SDL_S(x, 19) ^ SDL_S(x, 61) ^ SDL_R(x, 6)) // we perform the left and right shift after that we will perform the XOR of all the results
#define SDL_BLOCK_SIZE  (uint8_t)128 //Size of each block which will be processed;
#define SDL_MIN(x, y) ( ((x)<(y))?(x):(y) )
#define SDL_CONST64(x) (uint64_t)x


/**
@ingroup  SDL_ROM_CHECKSUM_API
@defgroup SDL_ROM_CHECKSUM_IP_API ROM_CHECKSUM Low-Level API
 */
/**
@defgroup SDL_IP_ROM_CHECKSUM_DATASTRUCT  ROM_Checksum Data Structures
@ingroup SDL_ROM_CHECKSUM_IP_API
*/

/**
@defgroup SDL_IP_ROM_CHECKSUM_FUNCTION  ROM Checksum Functions
@ingroup SDL_ROM_CHECKSUM_IP_API
*/

/**
 *  @addtogroup SDL_IP_ROM_CHECKSUM_DATASTRUCT
    @{
 *
 */

/** ---------------------------------------------------------------------------
 * @brief   This structure is used to store the resultant value of ROM Checksum
 *
 * ----------------------------------------------------------------------------
 */
typedef struct
{
	/** Actual data length*/
	uint64_t length;

	/** Random seek value*/
	uint64_t state[8];

	/** Used to keep track of data length in incomplete block*/
	uint32_t curlen;

	/** Memory storage use in padding*/
	uint8_t  buf[128];
} SDL_ROM_Checksum_obj;

/** @} */
/********************************************************************************************************
*   Below are the Declarations of Low Level Functions
********************************************************************************************************/

/**
 *  @addtogroup SDL_IP_ROM_CHECKSUM_FUNCTION
    @{
 *
 */
/**
* \brief This API will Initialize the buffer where hash value to be stored.
*
*/
void SDL_ROM_Checksum_init(SDL_ROM_Checksum_obj * const md);

/**
* \brief This API is used to process the ROM region data.
*
* \return status Success of the ROM Checksum.
*      Success: SDL_PASS.
*      Fail : SDL_EFAIL.
*
*/
int32_t SDL_ROM_Checksum_process (SDL_ROM_Checksum_obj * md,  uint8_t *in, uint32_t inlen);

/**
* \brief This API is used to compress the data of ROM region and store the result in md->state.
*
*/
void SDL_ROM_Checksum_compress(SDL_ROM_Checksum_obj * md, uint8_t *buf);

/**
* \brief This API will increase our data length with the help of padding ( because our algorithm can compress only 1024-bit length of data at a time)  and will compress then will store final has value in md->state.
* \return status Success of the ROM Checksum.
*      Success: SDL_PASS.
*      Fail : SDL_EFAIL.
*
*/
int32_t SDL_ROM_Checksum_done(SDL_ROM_Checksum_obj * md);


/**
* \brief This API will compare the resultant hash value of golden value.
*
* \return status Success of the ROM Checksum.
*     Success: SDL_PASS.
*     Fail : SDL_EFAIL.
*
*/
int32_t SDL_ROM_Checksum_compareResult (uint64_t buflen, SDL_ROM_Checksum_obj * md,  uint64_t *golden_value);

/**
* \brief This API will copy source pointer data to destination pointer.
*
*/
void SDL_memcpy(void *dest, void *src, uint32_t n);

/** @} */

#ifdef __cplusplus
}
#endif
#endif /*  _sdl_ip_rom_checksum_H */
