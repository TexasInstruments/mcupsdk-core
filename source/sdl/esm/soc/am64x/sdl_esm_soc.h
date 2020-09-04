/*
 * SDL ESM
 *
 * Software Diagnostics Reference module for Error Signaling Module
 *
 *  Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef INCLUDE_SDL_ESM_SOC_AM64X_H_
#define INCLUDE_SDL_ESM_SOC_AM64X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/include/am64x_am243x/sdlr_intr_mcu_esm0.h>
#include <sdl/include/am64x_am243x/sdlr_intr_esm0.h>
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>

/** ---------------------------------------------------------------------------
 * \brief Defines the different ESM instance types
 * ----------------------------------------------------------------------------
 */
typedef enum {
    /**< MCU_ESM0 instance  */
   SDL_ESM_INST_MCU_ESM0 = 1,
   SDL_ESM_INST_MAIN_ESM0 = 2,       
    /**< ESM0 (Main domain) instance  */
   SDL_ESM_INSTANCE_MAX=0xFFFF
} SDL_ESM_Inst;

/* Defines for ESM base addresses */
#define SOC_MCU_ESM_BASE  (SDL_MCU_ESM0_CFG_BASE)
#define SOC_MAIN_ESM_BASE (SDL_ESM0_CFG_BASE)

#define SOC_MCU_ESM_MAX_NUM_EVENTS (SDLR_MCU_ESM0_ESM_PLS_EVENT2_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_11+1U)
#define SOC_MAIN_ESM_MAX_NUM_EVENTS (SDLR_ESM0_ESM_PLS_EVENT2_COMPUTE_CLUSTER0_PBIST_0_DFT_PBIST_CPU_0+1U)

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif /* INCLUDE_SDL_ESM_SOC_AM64X_H_ */
