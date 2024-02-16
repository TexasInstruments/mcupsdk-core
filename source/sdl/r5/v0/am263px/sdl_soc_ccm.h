/********************************************************************
*
* SOC CCM PROPERTIES. header file
*
*  Copyright (c) Texas Instruments Incorporated 2022-2024
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
 *  @addtogroup SDL_CCM_MODULE APIs for CCM MODULE
    @{
 */

#ifndef SDL_SOC_CCM_H_
#define SDL_SOC_CCM_H_
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <sdl/sdlr.h>
#include <sdl/include/am263px/sdlr_soc_baseaddress.h>
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/esm/soc/am263px/sdl_esm_core.h>
#include <sdl/include/am263px/sdlr_intr_esm0.h>
#ifdef __cplusplus
extern "C"
{
#endif
/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_CCM_ESM_U_BASE      (SDL_ESM0_CCM_0_SELFTEST_ERR)
#define SDL_INTR_GROUP_NUM      1U
#define BASEADDRESS 			SDL_R5SS0_CCMR_U_BASE
#define ESM_INSTANCE 			SDL_ESM_INST_MAIN_ESM0

/**
 * \brief  CCM Instance supported.
 */

typedef uint32_t SDL_CCM_Inst;
	/** CCM INSTANCE R5F0 */
#define SDL_R5SS0_CCM           0U
    /** CCM INSTANCE R5F1 */
#define SDL_R5SS1_CCM           1U
	/** Maximum */
#define SDL_CCM_MAX_INSTANCE  (SDL_R5SS1_CCM + 1U)

/**
 *
 * \brief   This API is used to get the base address of the instance.
 *
 */

static uint32_t SDL_CCM_baseAddress[SDL_CCM_MAX_INSTANCE] =
{
	SDL_R5SS0_CCMR_U_BASE,
    SDL_R5SS1_CCMR_U_BASE,
};

#ifdef __cplusplus
}
#endif
#endif /* SDL_SOC_CCM_H_ */
 /** @} */
