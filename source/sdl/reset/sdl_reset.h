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
 *  Name        : sdl_reset.h
*/
/**
 *
 *  \defgroup SDL_RESET_API SDL RESET IMPLEMENTATION
 *  \ingroup SDL_RESET_MODULE
 *
 *   Provides the APIs for RESET.
 *  @{
 */
/**
 *  \file     sdl_reset.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of RESET.
 *            This also contains some related macros.
 */


#ifndef SDL_RESET_H_
#define SDL_RESET_H_

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
#include <sdl/sdlr.h>
#include <sdl/reset/soc/sdl_soc_reset.h>
#include <sdl/soc.h>


/**
@defgroup SDL_RESET_MACROS RESET Macros
@ingroup SDL_RESET_API
*/

/**
@defgroup SDL_RESET_ENUM RESET Enumerated Data Types
@ingroup SDL_RESET_API
*/

/**
@defgroup SDL_RESET_FUNCTIONS RESET Functions
@ingroup SDL_RESET_API
*/

/**************************************************************************
* RESET MACROS :
**************************************************************************/
/**

@addtogroup SDL_RESET_MACROS
@{
*/


/** @} */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */


/**

@addtogroup SDL_RESET_ENUM
@{
*/

/**
 *  \anchor SOC_WarmResetCause_e
 *  \name SOC Warm Reset Causes
 *  @{
 */
typedef enum SDL_SOC_WarmResetCause_e
{
    /**
     * \brief Value specifying Power ON Reset
     */
    SDL_WarmResetCause_POWER_ON_RESET = 0x09U,
    /**
     * \brief Value specifying MSS WDT
     */
    SDL_WarmResetCause_MSS_WDT = 0x0AU,
    /**
     * \brief Value specifying Software Warm Reset
     */
    SDL_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG = 0x0CU,
    /**
     * \brief Value specifying External Pad Reset
     */
    SDL_WarmResetCause_EXT_PAD_RESET = 0x08U,
    /**
     * \brief Value specifying HSM WDT
     */
    SDL_WarmResetCause_HSM_WDT = 0x18U,

}SDL_SOC_WarmResetCause;
/** @} */
/** @} */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_RESET_FUNCTIONS
@{
*/

/**
 *  \brief Get R5 reset cause
 *
 * \return R5 reset cause
 */
uint32_t  SDL_r5fGetResetCause(void);

/**
 * \brief Generate SW WARM reset
 */
void SDL_generateSwWarmReset(void);

/**
 * \brief Returns cause of WARM reset
 *
 * \return cause of WARM reset.
 */
uint32_t SDL_getWarmResetCause(void);

/**
 *  \brief Reset Dsp Core
 */
void SDL_rcmDspLocalReset(void);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
#endif /* SDLR_RESET_H_ */
