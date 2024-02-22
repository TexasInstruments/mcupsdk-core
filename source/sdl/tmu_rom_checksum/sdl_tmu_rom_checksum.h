/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  Name        : sdl_tmu_rom_checksum.h
*/
/**
 *
 *  \defgroup SDL_TMU_ROM_CHECKSUM_API SDL TMU ROM CHECKSUM IMPLEMENTATION
 *  \ingroup SDL_TMU_ROM_CHECKSUM_MODULE
 *
 *   Provides the APIs for CRC for TMU ROM.
 *  @{
 */
/**
 *  \file     sdl_tmu_rom_checksum.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of TMU ROM checksum.
 *            This also contains some related macros.
 */


#ifndef SDL_TMU_ROMCHKSUM_H_
#define SDL_TMU_ROMCHKSUM_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#include <stdbool.h>
#include <stdint.h>
#include <sdl/include/hw_types.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/sdlr.h>
#include <sdl/soc.h>
#include <sdl/sdl_mcrc.h>

/**
@defgroup SDL_ROMCHECKSUM_MACROS TMU ROM Checksum Macros
@ingroup SDL_TMU_ROM_CHECKSUM_API
*/

/**
@defgroup SDL_ROMCHECKSUM_ENUM TMU ROM Checksum Enumerated Data Types
@ingroup SDL_TMU_ROM_CHECKSUM_API
*/

/**
@defgroup SDL_ROMCHECKSUM_FUNCTIONS TMU ROM Checksum Functions
@ingroup SDL_TMU_ROM_CHECKSUM_API
*/

/**************************************************************************
* TMU ROM Checksum MACROS :
**************************************************************************/
/**

@addtogroup SDL_ROMCHECKSUM_MACROS
@{
*/


#define SDL_TMU_CRC_GOLDEN_REFL (0x36CFB524U)
#define SDL_TMU_CRC_GOLDEN_REFH (0xCCDAB254U)
#define SDL_TMU_MCRC_PCNT       (0U)
#define SDL_TMU_MCRC_SCNT       (1U)
#define SDL_TMU_MCRC_WDPRELD    (0U)
#define SDL_TMU_MCRC_BLKPRELD   (0U)

#define SDL_R5F_CORE           (0U)  /* Options  0,1,2,3 */
#define SDL_DEF_TMU_ROM_BA     (0x53020000U + (SDL_R5F_CORE * 0x4000U))
#define SDL_TMU_ROM_SIZE       (0x3000U)  /* 12KB */

/** @} */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_TMU_ROMCHECKSUM_FUNCTIONS
@{
*/

/**
 *  \brief Compute TMU Rom Checksum
 *
 * \param   mcrcChannelNumber       MCRC channel number.
 * \param   sectSignVal             Pointer to return the calculated signature
 *
 * \return  status          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input
 *                                        arguments
 *                          SDL_EFAIL: failure, indicate CRC signature did not
 *                                     match
 */
int32_t SDL_TMU_ROM_Checksum_compute(SDL_MCRC_Channel_t mcrcChannelNumber,
                                           SDL_MCRC_Signature_t  *sectSignVal);


/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
#endif /* SDL_TMU_ROMCHKSUM_H_ */
