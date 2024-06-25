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

#ifndef INCLUDE_SDL_ESM_SOC_AWR294X_H_
#define INCLUDE_SDL_ESM_SOC_AWR294X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/include/awr294x/sdlr_intr_dss.h>
#include <sdl/include/awr294x/sdlr_intr_mss.h>
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
/** ---------------------------------------------------------------------------
 * \brief Defines the different ESM instance types
 * ----------------------------------------------------------------------------
 */
typedef enum {
    /**< MSS_ESM instance  */
   SDL_ESM_INST_MSS_ESM = 1U,
       /**< MSS_ESM instance  */
   SDL_ESM_INST_DSS_ESM = 2U,
       /**< HSM_ESM instance  */
   SDL_ESM_INST_HSM_ESM = 3U,   
	/**< Invalid_ESM instance  */
   SDL_ESM_INSTANCE_MAX=0xFFFFU
} SDL_ESM_Inst;

/* Defines for ESM base addresses */
#define SOC_MSS_ESM_BASE  (SDL_MSS_ESM_U_BASE)
#define SOC_DSS_ESM_BASE (SDL_DSS_ESM_U_BASE)
#define SOC_HSM_ESM_BASE (SDL_HSM_ESM_U_BASE)

/* Enumerate Interrupt number for the different esm interrupts */
#define SDL_MSS_ESM_HI_INTNO  SDL_MSS_INTR_MSS_ESM_HI
#define SDL_MSS_ESM_LOW_INTNO  SDL_MSS_INTR_MSS_ESM_LO 

#define SDL_DSS_ESM_HI_INTNO  SDL_DSS_INTR_DSS_ESM_HI
#define SDL_DSS_ESM_LOW_INTNO  SDL_DSS_INTR_DSS_ESM_LO

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif /* INCLUDE_SDL_ESM_SOC_AWR294X_H_ */
