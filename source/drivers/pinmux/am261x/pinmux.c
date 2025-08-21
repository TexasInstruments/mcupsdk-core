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

/* define the unlock and lock values for IOMUX*/
#define IOMUX_KICK_LOCK_VAL                     (0x00000000U)
#define IOMUX_KICK0_UNLOCK_VAL                  (0x83E70B13U)
#define IOMUX_KICK1_UNLOCK_VAL                  (0x95A4F1E0U)
#define IOMUX_USER_MODE_ENABLE_VAL              (0xADADADADU)
#define IOMUX_USER_MODE_DISABLE_VAL             (0x00000000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void Pinmux_lockMMR(uint32_t domainId);
static void Pinmux_unlockMMR(uint32_t domainId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId)
{
    volatile uint32_t  *regAddr;
    if(NULL != pinmuxCfg)
    {
        Pinmux_unlockMMR(domainId);
        while( pinmuxCfg->offset != PINMUX_END )
        {
            regAddr = (volatile uint32_t *)((uint32_t)CSL_IOMUX_U_BASE + pinmuxCfg->offset);
            CSL_REG32_WR(regAddr, pinmuxCfg->settings);
            pinmuxCfg++;
        }
        Pinmux_lockMMR(domainId);
    }
    return;
}

static void Pinmux_lockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    /*Lock IOMUX*/
    baseAddr = (uint32_t) CSL_IOMUX_U_BASE;
    kickAddr = (volatile uint32_t *) (baseAddr + CSL_IOMUX_IO_CFG_KICK0);
    CSL_REG32_WR(kickAddr, IOMUX_KICK_LOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + CSL_IOMUX_IO_CFG_KICK1);
    CSL_REG32_WR(kickAddr, IOMUX_KICK_LOCK_VAL);      /* KICK 1 */

    return;
}

static void Pinmux_unlockMMR(uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    /*Unlock IOMUX*/
    baseAddr = (uint32_t) CSL_IOMUX_U_BASE;
    kickAddr = (volatile uint32_t *) (baseAddr + CSL_IOMUX_IO_CFG_KICK0);
    CSL_REG32_WR(kickAddr, IOMUX_KICK0_UNLOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + CSL_IOMUX_IO_CFG_KICK1);
    CSL_REG32_WR(kickAddr, IOMUX_KICK1_UNLOCK_VAL);      /* KICK 1 */

    return;
}

void Pinmux_qualPeriodConfig(uint32_t qualGroupIndex, uint8_t qualPeriod)
{
    volatile uint32_t  *regAddr;

    if(qualGroupIndex <= PIN_QUAL_GRP_MAX_INDEX)
    {
        Pinmux_unlockMMR(PINMUX_DOMAIN_ID_MAIN);
        regAddr = (volatile uint32_t *)((uint32_t)CSL_IOMUX_U_BASE + 
                   PIN_QUAL_GROUP_CFG_START + ((uint32_t)(qualGroupIndex * 4U)));
        CSL_REG32_WR(regAddr, (uint32_t)qualPeriod);
        Pinmux_lockMMR(PINMUX_DOMAIN_ID_MAIN);
    }
    return;
}

void Pinmux_enableUserMode(void)
{
    volatile uint32_t  *userModeReg;

    /* Enable user mode */
    userModeReg = (volatile uint32_t *) ((uint32_t)(CSL_IOMUX_U_BASE + CSL_IOMUX_USER_MODE_EN));
    CSL_REG32_WR(userModeReg, IOMUX_USER_MODE_ENABLE_VAL);

    return;
}

void Pinmux_disableUserMode(void)
{
    volatile uint32_t  *userModeReg;

    /* Disable user mode */
    userModeReg = (volatile uint32_t *) ((uint32_t)(CSL_IOMUX_U_BASE + CSL_IOMUX_USER_MODE_EN));
    CSL_REG32_WR(userModeReg, IOMUX_USER_MODE_DISABLE_VAL);

    return;
}