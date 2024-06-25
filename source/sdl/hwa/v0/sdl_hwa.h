/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  @defgroup SDL_HWA_MODULE_API API's for HWA  memory Parity check and FSM lockstep
 *  @ingroup  SDL_HWA_MODULE
 *  @section  SDL_HWA Overview
 *
 *   Provides the APIs for HWA memory Parity check and FSM lockstep
 *  @{
 */
/**
 *  \file     sdl_hwa.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of HWA.
 *            This also contains some related macros.
 */

#ifndef SDL_HWA_H_
#define SDL_HWA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_hwa_hw.h"
#include "sdl_ip_hwa.h"
#include <sdl/hwa/v0/soc/sdl_hwa_soc.h>
#include <sdl/esm/sdlr_esm.h>
#if defined (SOC_AM273X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/am273x/sdlr_intr_esm_dss.h>
#include <sdl/include/am273x/sdlr_intr_esm_mss.h>
#elif defined (SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/awr294x/sdlr_intr_esm_dss.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#endif

#ifdef _cplusplus
extern "C" {
#endif
/**
 @defgroup SDL_HWA_MACROS HWA Macros
 @ingroup SDL_HWA_MODULE_API
*/

/**
 @defgroup SDL_HWA_FUNCTIONS HWA Functions
 @ingroup SDL_HWA_MODULE_API
*/

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 @addtogroup SDL_HWA_MACROS
@{
*/

#define SDL_HWA_INIT_TIMEOUT   (0XFFU)


/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 @addtogroup SDL_HWA_FUNCTIONS
@{
*/

/**
 * \brief   This API is a call back function when an interrupt is occured for HWA memory parity and fsm Lockstep
 *
 * \param   instance   indicates the ESM instance
 *
 * \param   grpChannel indicates the ESM channel number for HWA memory parity and fsm Lockstep
 *
 * \param   vecNum     indicates the ESM vector number for HWA memory parity and fsm Lockstep
 *
 * \param   arg        pointer to input arrguments
 *
 * \return  status    return the status for callback
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */

int32_t SDL_HWA_ESM_CallbackFunction(SDL_ESM_Inst instance,
                                     int32_t grpChannel,
                                     int32_t vecNum,
                                     void *arg);

/**
 * \brief   This API is used  for configuring and testing parity of the HWA memory
 *
 * \param   memID     indicates the memory in the HWA
 *
 * \param   memBlock  indicates the memory block in the HWA memory
 *
 * \return  status    return the base address of th instance.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */

int32_t SDL_HWA_memParityExecute( SDL_HWA_MemID memID , SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to induce the error in the fsm lockstep for HWA
 *
 *
 * \return  status    return the base address of th instance.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */
int32_t SDL_HWA_fsmLockStepExecute( void);

/** @} */


#ifdef _cplusplus
}
#endif /*extern "C" */
#endif

/** @} */
