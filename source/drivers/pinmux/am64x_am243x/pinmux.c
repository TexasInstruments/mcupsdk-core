/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file   pinmux.c
 *
 *  \brief  PINMUX Driver file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/pinmux.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET   (0x1008)
#define CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET   (0x5008)
#define CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET    (0x1008)
#define CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET    (0x5008)

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x68EF3490U)
#define KICK1_UNLOCK_VAL                        (0xD172BC5AU)

#define PADCFG_PMUX_OFFSET                      (0x4000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void Pinmux_lockMMR(uint32_t domainId);
void Pinmux_unlockMMR(uint32_t domainId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *regAddr;
    uint32_t            isUnlocked = 0;

    if((NULL != pinmuxCfg) && (pinmuxCfg->offset != PINMUX_END))
    {
        if(PINMUX_DOMAIN_ID_MAIN == domainId)
        {
            baseAddr = CSL_PADCFG_CTRL0_CFG0_BASE + PADCFG_PMUX_OFFSET;
        }
        else
        {
            baseAddr = CSL_MCU_PADCFG_CTRL0_CFG0_BASE + PADCFG_PMUX_OFFSET;
        }
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(baseAddr);
        if (pinmuxCfg->offset != PINMUX_END)
        {
            isUnlocked = 1;
            Pinmux_unlockMMR(domainId);
        }
        while( pinmuxCfg->offset != PINMUX_END )
        {
            regAddr = (volatile uint32_t *)(baseAddr + pinmuxCfg->offset);
            CSL_REG32_WR(regAddr, pinmuxCfg->settings);
            pinmuxCfg++;
        }
        if (isUnlocked)
        {
            Pinmux_lockMMR(domainId);
        }
    }
    return;
}

void Pinmux_lockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(PINMUX_DOMAIN_ID_MAIN == domainId)
    {
        #if 0 /* in AM64x, main dowmin MMRs are left unlocked since when working with linux kernel, linux kernel assumes MMRs are unlocked */
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_PADCFG_CTRL0_CFG0_BASE);
        /* Lock 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */

        /* Lock 1 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
        #endif
    }

    if(PINMUX_DOMAIN_ID_MCU == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_PADCFG_CTRL0_CFG0_BASE);
        /* Lock 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */

        /* Lock 1 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    return;
}

void Pinmux_unlockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(PINMUX_DOMAIN_ID_MAIN == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_PADCFG_CTRL0_CFG0_BASE);
        /* Lock 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */

        /* Lock 1 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(PINMUX_DOMAIN_ID_MCU == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_PADCFG_CTRL0_CFG0_BASE);
        /* Lock 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */

        /* Lock 1 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    return;
}
