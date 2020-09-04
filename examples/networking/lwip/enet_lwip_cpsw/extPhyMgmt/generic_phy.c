/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file  generic_phy.c
 *
 * \brief This file contains the implementation of the generic Ethernet PHY.
 *        It provides the basic functionality allowed with IEEE standard
 *        registers.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdlib.h>
#include "enetextphy.h"
#include "enetextphy_priv.h"
#include "generic_phy.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool GenericExtPhy_isPhyDevSupported(EnetExtPhy_Handle hPhy,
                                         const EnetExtPhy_Version *version);

static bool GenericExtPhy_isMacModeSupported(EnetExtPhy_Handle hPhy,
                                          EnetExtPhy_Mii mii);

static void GenericExtPhy_printRegs(EnetExtPhy_Handle hPhy);
static void GenericExtPhy_reset(EnetExtPhy_Handle hPhy);
static bool GenericExtPhy_isResetComplete(EnetExtPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetExtPhy_Drv gEnetExtPhyDrvGeneric =
{
    .name               = "generic",
    .isPhyDevSupported  = GenericExtPhy_isPhyDevSupported,
    .isMacModeSupported = GenericExtPhy_isMacModeSupported,
    .config             = NULL,
    .reset              = GenericExtPhy_reset,
    .isResetComplete    = GenericExtPhy_isResetComplete,
    .readExtReg         = GenericExtPhy_readExtReg,
    .writeExtReg        = GenericExtPhy_writeExtReg,
    .printRegs          = GenericExtPhy_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static bool GenericExtPhy_isPhyDevSupported(EnetExtPhy_Handle hPhy,
                                         const EnetExtPhy_Version *version)
{
    /* All IEEE-standard PHY models are supported */
    return true;
}

static bool GenericExtPhy_isMacModeSupported(EnetExtPhy_Handle hPhy,
                                          EnetExtPhy_Mii mii)
{
    /* All MAC modes are supported */
    return true;
}

static void GenericExtPhy_reset(EnetExtPhy_Handle hPhy)
{
    ENETEXTPHYTRACE_DBG("PHY %u: reset\n", hPhy->addr);

    /* Reset the PHY */
    EnetExtPhy_rmwReg(hPhy, PHY_BMCR, BMCR_RESET, BMCR_RESET);
}

static bool GenericExtPhy_isResetComplete(EnetExtPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bit has self-cleared */
    status = EnetExtPhy_readReg(hPhy, PHY_BMCR, &val);
    if (status == ENETEXTPHY_SOK)
    {
        complete = ((val & BMCR_RESET) == 0U);
    }

    ENETEXTPHYTRACE_DBG("PHY %u: reset is %scomplete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

int32_t GenericExtPhy_readExtReg(EnetExtPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t *val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    int32_t status;

    status = EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);

    if (status == ENETEXTPHY_SOK)
    {
        status = EnetExtPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETEXTPHY_SOK)
    {
        EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETEXTPHY_SOK)
    {
        status = EnetExtPhy_readReg(hPhy, PHY_MMD_DR, val);
    }

    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: failed to read reg %u\n", hPhy->addr, reg);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: read reg %u val 0x%04x\n", hPhy->addr, reg, *val);

    return status;
}

int32_t GenericExtPhy_writeExtReg(EnetExtPhy_Handle hPhy,
                               uint32_t reg,
                               uint16_t val)
{
    uint16_t devad = MMD_CR_DEVADDR;
    int32_t status;

    ENETEXTPHYTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n", hPhy->addr, reg, val);

    status = EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == ENETEXTPHY_SOK)
    {
        EnetExtPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETEXTPHY_SOK)
    {
        EnetExtPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETEXTPHY_SOK)
    {
        EnetExtPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", hPhy->addr, reg, val);

    return status;
}

static void GenericExtPhy_printRegs(EnetExtPhy_Handle hPhy)
{
    uint32_t i;
    uint16_t val;

    for (i = PHY_BMCR; i <= PHY_GIGESR; i++)
    {
        EnetExtPhy_readReg(hPhy, i, &val);
        DebugP_log("PHY %u: reg 0x%02x = 0x%04x\n", hPhy->addr, i, val);
    }
}
