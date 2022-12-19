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

#ifndef INCLUDE_SDL_ESM_CORE_H_
#define INCLUDE_SDL_ESM_CORE_H_
#include <sdl/esm/v0/sdl_esm.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/include/am64x_am243x/sdlr_intr_mcu_m4fss0_core0.h>
#include <sdl/include/am64x_am243x/sdlr_intr_r5fss0_core0.h>


/* Enumerate Interrupt number for the different esm interrupts */
#if defined (M4F_CORE)
#define SDL_MCU_ESM_HI_INTNO  SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_HI_LVL_0
#define SDL_MCU_ESM_LO_INTNO  SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_LOW_LVL_0 
#define SDL_MCU_ESM_CFG_INTNO SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_CFG_LVL_0
#endif

#if defined (R5F_CORE)
#define SDL_MAIN_ESM_HI_INTNO  SDLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI_LVL_0
#define SDL_MAIN_ESM_LO_INTNO  SDLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW_LVL_0
#define SDL_MAIN_ESM_CFG_INTNO SDLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_CFG_LVL_0


#endif

#ifdef __cplusplus
}
#endif  /* extern "C" */
#endif /* INCLUDE_SDL_ESM_CORE_H_ */
