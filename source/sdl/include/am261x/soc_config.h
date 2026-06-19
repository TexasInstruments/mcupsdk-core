/*
 *  Copyright (C) 2022-2025 Texas Instruments Incorporated
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

#ifndef SDL_SOC_CONFIG_IN_H_
#define SDL_SOC_CONFIG_IN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <sdl/include/am261x/sdlr_mss_ctrl.h>
#include <sdl/include/am261x/sdlr_intr_r5fss0_core0.h>
#include <sdl/include/am261x/sdlr_intr_r5fss0_core1.h>
#include <sdl/include/am261x/sdlr_soc_baseaddress.h>

/* IP versions */
#define IP_VERSION_MCRC_V0
#define IP_VERSION_ESM_V2_0
#define IP_VERSION_DCC_V1
#define IP_VERSION_RTI_V0
#define IP_VERSION_PBIST_V0
#define IP_VERSION_ECC_BUS_SAFETY_V0
#define IP_VERSION_STC_V0
#define IP_VERSION_ECC_V1
#define IP_VERSION_TMU_ROMCHECKSUM_V0
#define IP_VERSION_TOG_V0
#define IP_VERSION_VTM_V1

/* Unlock and Lock values for MSS_CTRL, TOP_CTRL, MSS_RCM and TOP_RCM */
#define SDL_LOCK0_KICK0             (0x00001008U)
#define SDL_LOCK0_KICK1             (0x0000100CU)
#define SDL_KICK_LOCK_VAL           (0x00000000U)
#define SDL_KICK0_UNLOCK_VAL        (0x01234567U)
#define SDL_KICK1_UNLOCK_VAL        (0x0FEDCBA8U)

/* baseAddr - MSS_CTRL, TOP_CTRL, MSS_RCM and TOP_RCM*/
void SDL_MMR_Unlock(uint32_t baseAddr);
void SDL_MMR_Lock(uint32_t baseAddr);

#ifdef __cplusplus
}
#endif


#endif
