/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  @addtogroup SDL_MSS_CR5_API API's for MSS CR5
    @{
 */

#ifndef SDL_MSS_CR5_SOC_H_
#define SDL_MSS_CR5_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am263x/sdlr_soc_baseaddress.h>
#include <sdl/include/am263x/sdlr_mss_ctrl.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_ECC_BUS_SAFETY_MSS_BUS_CFG           (uint32_t)SDL_MSS_CTRL_U_BASE

#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE   (0x000000A0U)
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE   (0x000000A4U)
#define SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE   (0x000000B0U)
#define SDL_MSS_CTRL_R5SS1_CORE1_AHB_BASE   (0x000000B4U)
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_END   (SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_END   (SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE + SDL_MSS_CTRL_R5SS1_CORE0_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS0_CORE1_AHB_END   (SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS1_CORE1_AHB_END   (SDL_MSS_CTRL_R5SS1_CORE1_AHB_BASE + SDL_MSS_CTRL_R5SS1_CORE1_AHB_SIZE)

#define SDL_R5SS0_CORE0_TCMA_U_SIZE (0x000000020)
#define SDL_R5SS0_CORE0_TCMB_U_SIZE (0x000000020)
#define SDL_MSS_CR5A_TCM_U_BASE (SDL_R5SS0_CORE0_TCMA_U_BASE )
#define SDL_MSS_CR5B_TCM_U_BASE (SDL_R5SS0_CORE0_TCMB_U_BASE )
#define SDL_MSS_CR5A_TCM_U_END  (SDL_R5SS0_CORE0_TCMA_U_BASE + SDL_R5SS0_CORE0_TCMA_U_SIZE)
#define SDL_MSS_CR5B_TCM_U_END  (SDL_R5SS0_CORE0_TCMB_U_BASE + SDL_R5SS0_CORE0_TCMB_U_SIZE)
#define SDL_MBOX_SRAM_U_BASE_END           (SDL_MBOX_SRAM_U_BASE+100U)

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
