/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file     sdl_reset.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of RESET.
 *            This also contains some related macros and Helper APIs.
 */

 /**
 * Design: PROC_SDL-5821
 */

#include <sdl/reset/sdl_reset.h>

/********************************************************************************************************
*   API for genrate Warm reset.
*********************************************************************************************************/

/* [PROC_SDL-3641] */
void SDL_generateSwWarmReset(void)
{
    /* Unlock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleUnlockMMR(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM);

    /* Assert SW Warm Reset */
    HW_WR_FIELD32(SDL_MSS_TOPRCM_U_BASE + SDL_MSS_TOPRCM_WARM_RESET_CONFIG, SDL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST, 0x0U);

    /* Lock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleLockMMR(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM);
}

/********************************************************************************************************
*   API for getting the status of warm reset cause
*********************************************************************************************************/

/* [PROC_SDL-3642] */
uint32_t SDL_getWarmResetCause(void)
{
    uint32_t     resetCause = 0U;

    /* Unlock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleUnlockMMR(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM);

    /* Read the Reset Cause Register bits */
    resetCause = (uint32_t)HW_RD_FIELD32 (SDL_MSS_TOPRCM_U_BASE + SDL_MSS_TOPRCM_SYS_RST_CAUSE, SDL_MSS_TOPRCM_SYS_RST_CAUSE_SYS_RST_CAUSE_CAUSE);

    /* clear the reset cause */
    HW_WR_FIELD32(SDL_MSS_TOPRCM_U_BASE + SDL_MSS_TOPRCM_SYS_RST_CAUSE_CLR, SDL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR, 0x1U);

    /* Lock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleLockMMR(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM);

    return  resetCause;
}

/********************************************************************************************************
*   API for getting the status of R5F Core reset cause
*********************************************************************************************************/

/* [PROC_SDL-3640]  */
uint32_t SDL_r5fGetResetCause(void)
{
    uint32_t     resetCauseBits = 0U;
    uint32_t     warmResetCause;

    /* Unlock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleUnlockMMR(SDL_SOC_DOMAIN_ID_MSS_RCM);

    /* Read the Reset Cause Register bits */
    resetCauseBits = (uint32_t)HW_RD_FIELD32 (SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_MSS_RST_STATUS, SDL_MSS_RCM_MSS_RST_STATUS_MSS_RST_STATUS_CAUSE);

    /* Reading the warm reset bit only */
    resetCauseBits = resetCauseBits >> 1U;
    warmResetCause = ((resetCauseBits & 0x1U));

    /* clear the reset cause */
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_MSS_RST_CAUSE_CLR, SDL_MSS_RCM_MSS_RST_CAUSE_CLR_MSS_RST_CAUSE_CLR_CLR, 0x7U);

    /* Lock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleLockMMR(SDL_SOC_DOMAIN_ID_MSS_RCM);

    return warmResetCause;
}


/********************************************************************************************************
*   API to generate localized reset for C66x DSP
*********************************************************************************************************/

/* [PROC_SDL-3643]  */

void SDL_rcmDspLocalReset(void)
{
    /* Unlock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleUnlockMMR(SDL_SOC_DOMAIN_ID_DSS_RCM);

    /* Assert DSP Local Reset */
    HW_WR_FIELD32(SDL_DSS_RCM_U_BASE  + SDL_DSS_RCM_DSS_DSP_RST_CTRL , SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL, 0x7U);

    /* Lock CONTROLSS_CTRL registers */
    SDL_SOC_controlModuleLockMMR(SDL_SOC_DOMAIN_ID_DSS_RCM);
}

/* Nothing past this point */
