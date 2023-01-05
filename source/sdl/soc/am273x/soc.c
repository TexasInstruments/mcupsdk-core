/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 */


#include "soc.h"


/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x01234567U)
#define KICK1_UNLOCK_VAL                        (0x0FEDCBA8U)
#define IOMUX_KICK0_UNLOCK_VAL                  (0x83E70B13U)
#define IOMUX_KICK1_UNLOCK_VAL                  (0x95A4F1E0U)

void SDL_SOC_controlModuleLockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM == domainId)
    {
        baseAddr = SDL_MSS_TOPRCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_TOPRCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_TOPRCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_MSS_RCM == domainId)
    {
        baseAddr = SDL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_DSS_RCM == domainId)
    {
        baseAddr = SDL_DSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_MSS_CTRL == domainId)
    {
        baseAddr = SDL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_CTRL_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_CTRL_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_DSS_CTRL == domainId)
    {
        baseAddr = SDL_DSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_CTRL_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_CTRL_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    return;
}

void SDL_SOC_controlModuleUnlockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SDL_SOC_DOMAIN_ID_MSS_TOP_RCM == domainId)
    {
        baseAddr = SDL_MSS_TOPRCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_TOPRCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_TOPRCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_MSS_RCM == domainId)
    {
        baseAddr = SDL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_DSS_RCM == domainId)
    {
        baseAddr = SDL_DSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_MSS_CTRL == domainId)
    {
        baseAddr = SDL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_CTRL_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_MSS_CTRL_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SDL_SOC_DOMAIN_ID_DSS_CTRL == domainId)
    {
        baseAddr = SDL_DSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_CTRL_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_DSS_CTRL_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    return;
}

