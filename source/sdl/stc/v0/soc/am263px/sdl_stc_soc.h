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


#ifndef SDL_STC_SOC_H_
#define SDL_STC_SOC_H_

#include <sdl/include/am263px/sdlr_soc_baseaddress.h>
#include <sdl/include/am263px/sdlr_mss_rcm.h>
#include <sdl/include/am263px/sdlr_mss_ctrl.h>


#ifdef __cplusplus
extern "C" {
#endif

/** ===========================================================================
 *
 * @ingroup  SDL_STC_MODULE
 * @defgroup SDL_STC_API SDL Self Test Controller(STC)
 *
 * ============================================================================
 */
/**
@defgroup SDL_STC_ENUM STC Enumerated Data Types
@ingroup SDL_STC_API
*/

/**
@defgroup SDL_STC_FUNCTIONS STC Functions
@ingroup SDL_STC_API
*/

/*======================================================================================================
*   Required Macros for Instances config
======================================================================================================*/

#define SDL_STC_NUM_INSTANCES 2U

/** ===========================================================================
 *  @addtogroup SDL_STC_ENUM
    @{
 * ============================================================================
 */
/*======================================================================================================
*   Enum for Different Core in SOC
======================================================================================================*/

typedef enum {

    SDL_STC_INST_MAINR5F0,
    SDL_STC_INST_MAINR5F1,

    SDL_STC_INVALID_INSTANCE,

} SDL_STC_Inst;

/* @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_STC_FUNCTIONS
@{
*/
/**
 *
 *
 * \brief   This API is used to clear the reset cause bits for processor core.
 *
 */
static  void SDL_STC_resetCauseClearR5F0(void);
static  void SDL_STC_resetCauseClearR5F1(void);

/** @} */

/*======================================================================================================
*   This is global array gives the BASE ADDRESS of STC modules
======================================================================================================*/

static uint32_t SDL_STC_baseAddress[SDL_STC_NUM_INSTANCES] = {(uint32_t)SDL_R5SS0_STC_U_BASE, (uint32_t)SDL_R5SS1_STC_U_BASE};

#define MSS_RCM_BaseAddress         (SDL_MSS_RCM_U_BASE)
#define MSS_CTRL_BaseAddress        (SDL_MSS_CTRL_U_BASE)





#ifdef __cplusplus
}
#endif

#endif /* SDL_STC_SOC_H_ */
