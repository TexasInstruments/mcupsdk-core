/* Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  @file  sdl_rom_checksum.h
 *
 *  @brief This file contains the SDL ROM Checksum API's
 */

/**
 *   @ingroup SDL_MODULE
 *   @defgroup SDL_ROM_CHECKSUM_API ROM_CHECKSUM API
 *
 *   Provides the APIs for ROM Checksum.
 */
#ifndef SDL_ROM_CHECKSUM_H_
#define SDL_ROM_CHECKSUM_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <sdl/include/soc_config.h>
#if defined (IP_VERSION_ROMCHECKSUM_V0)
#include <sdl/rom_checksum/sdl_ip_rom_checksum.h>
#endif

/**
* @defgroup SDL_ROM_CHECKSUM_FUNCTION  ROM Checksum Functions
* @ingroup SDL_ROM_CHECKSUM_API
*/

/**
* @defgroup SDL_ROM_CHECKSUM_MACROS ROM Checksum MACROS
* @ingroup SDL_ROM_CHECKSUM_API
*/

/**
 *  @addtogroup SDL_ROM_CHECKSUM_MACROS
    @{
 *
 */

/**
 * \brief  Macro defines the length of data to be hashed.
 *          data to be hashed length is taken from known location of ROM.
 */
#define SDL_LENGTH_OF_DATA_TO_BE_HASHED ((uint32_t)196544)

/**
 * \brief  Macro defines the length of golden data.
 *         Golden data length is taken from known location of ROM.
 */
#define SDL_LENGTH_OF_GOLDEN_DATA ((uint64_t)8)

/**
 * \brief  Macro defines the pointer of golden data.
 *          Golden data pointer is taken from known location of ROM.
 */
#define SDL_GOLDEN_DATA_POINTER (uint64_t *)(0x004182FFC0)

/**
 * \brief  Macro defines the pointer of data to be hashed.
 *         data to be hashed pointer is  taken from known location of ROM.
 */
#define SDL_DATA_TO_BE_HASHED_POINTER (uint8_t *)(0x0041800000)

/** @} */

/**
 *  @addtogroup SDL_ROM_CHECKSUM_FUNCTION
    @{
 *
 */

/**
 *  \brief This function will compute Checksum of ROM.
 *
 *  \return The SDL error code for the API.
 *                          If failed: SDL_EFAIL
 *                          If pointer is invalid: SDL_EBADARGS
 *                          If  Length is Invalid: SDL_EBADARGS
 *                          Success: SDL_PASS
 *
 */
int32_t SDL_ROM_Checksum_compute (void);

/** @} */
#ifdef __cplusplus
}
#endif

#endif /* SDL_ROM_CHECKSUM_H_ */