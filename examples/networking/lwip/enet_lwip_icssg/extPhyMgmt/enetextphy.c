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
 * \file  enetextphy.c
 *
 * \brief This file contains the implementation of the Ethernet PHY driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "enetextphy.h"
#include "enetextphy_priv.h"
#include "generic_phy.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Capability string buffer length. */
#define ENETEXTPHY_CAPS_BUF_LEN                  (41U)

/*! \brief PHY mode string buffer length. */
#define ENETEXTPHY_MODE_BUF_LEN                  (25U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static const char *EnetExtPhy_getCapsString(uint32_t linkCaps);

static const char *EnetExtPhy_getModeString(EnetExtPhy_Speed speed,
                                         EnetExtPhy_Duplexity duplexity);

static EnetExtPhy_Handle EnetExtPhy_getHandle(void);

static void EnetExtPhy_releaseHandle(EnetExtPhy_Handle hPhy);

static void EnetExtPhy_initState(EnetExtPhy_Handle hPhy);

static void EnetExtPhy_enable(EnetExtPhy_Handle hPhy);

static void EnetExtPhy_setupNway(EnetExtPhy_Handle hPhy);

static bool EnetExtPhy_nwayStart(EnetExtPhy_Handle hPhy);

static bool EnetExtPhy_nwayWait(EnetExtPhy_Handle hPhy);

static bool EnetExtPhy_checkLink(EnetExtPhy_Handle hPhy);

static void EnetExtPhy_linkedState(EnetExtPhy_Handle hPhy);

static bool EnetExtPhy_isNwayCapable(EnetExtPhy_Handle hPhy);

static uint32_t EnetExtPhy_getLocalCaps(EnetExtPhy_Handle hPhy);

static uint32_t EnetExtPhy_findCommonCaps(EnetExtPhy_Handle hPhy);

static uint32_t EnetExtPhy_findCommon1000Caps(EnetExtPhy_Handle hPhy);

static uint32_t EnetExtPhy_findCommonNwayCaps(EnetExtPhy_Handle hPhy);

static bool EnetExtPhy_isPhyLinked(EnetExtPhy_Handle hPhy);

static int32_t EnetExtPhy_bindDriver(EnetExtPhy_Handle hPhy);

static void EnetExtPhy_resetPhy(EnetExtPhy_Handle hPhy);

static uint32_t EnetExtPhy_findBestCap(uint32_t caps);

static void EnetExtPhy_capToMode(uint32_t caps,
                              EnetExtPhy_Speed *speed,
                              EnetExtPhy_Duplexity *duplexity);

static void EnetExtPhy_showLinkPartnerCompat(EnetExtPhy_Handle hPhy,
                                          EnetExtPhy_Speed speed,
                                          EnetExtPhy_Duplexity duplexity);

static void EnetExtPhy_initState(EnetExtPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PHY drivers */
extern EnetExtPhy_Drv gEnetExtPhyDrvGeneric;
extern EnetExtPhy_Drv gEnetExtPhyDrvDp83867;
extern EnetExtPhy_Drv gEnetExtPhyDrvDp83869;


/*! \brief All the registered PHY specific drivers. */
static EnetExtPhyDrv_Handle gEnetExtPhyDrvs[] =
{
    &gEnetExtPhyDrvDp83867,   /* DP83867 */
    &gEnetExtPhyDrvDp83869,   /* DP83869 */
    &gEnetExtPhyDrvGeneric,   /* Generic PHY - must be last */
};

/*! \brief Enet MAC port objects. */
static EnetExtPhy_Obj gEnetExtPhy_phyObjs[ENETEXTPHY_PHY_MAX];

/*! \brief Ethernet PHY capabilities string buffer. */
static char gEnetExtPhyCapsBuf[ENETEXTPHY_CAPS_BUF_LEN];

/*! \brief Ethernet PHY mode string buffer. */
static char gEnetExtPhyModeBuf[ENETEXTPHY_MODE_BUF_LEN];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetExtPhy_initCfg(EnetExtPhy_Cfg *phyCfg)
{
    memset(phyCfg, 0, sizeof(*phyCfg));

    phyCfg->phyAddr = ENETEXTPHY_INVALID_PHYADDR;
    phyCfg->nwayCaps = ENETEXTPHY_LINK_CAP_ALL;
    phyCfg->mdixEn = true;
    phyCfg->masterMode = false;
    phyCfg->extClkSource = false;
    phyCfg->skipExtendedCfg = false;
}

void EnetExtPhy_setExtendedCfg(EnetExtPhy_Cfg *phyCfg,
                            const void *extendedCfg,
                            uint32_t extendedCfgSize)
{
    DebugP_assert(extendedCfgSize <= ENETEXTPHY_EXTENDED_CFG_SIZE_MAX);

    if ((extendedCfgSize > 0U) &&
        (extendedCfg != NULL))
    {
        memcpy(phyCfg->extendedCfg, extendedCfg, extendedCfgSize);
    }

    phyCfg->extendedCfgSize = extendedCfgSize;
}


static void EnetExtPhy_triggerReset(EnetExtPhy_Handle hPhy)
{
    bool complete;

    EnetExtPhy_resetPhy(hPhy);

    do {
        /* Wait for PHY reset to complete */
        complete = hPhy->hDrv->isResetComplete(hPhy);
    } while (complete != true);

}

bool EnetExtPhy_WaitForLinkUp(EnetExtPhy_Handle hPhy, uint32_t timeoutMs)
{
    bool isLinkUp = false;
    bool complete = false;
    uint32_t i;

    for (i = 0U; i < timeoutMs; i++)
    {
        complete =  EnetExtPhy_nwayStart(hPhy);
        if(complete)
        {
            break;
        }
        ClockP_usleep(1000);
    }

    if (complete)
    {
        for (i = 0U; i < timeoutMs; i++)
        {
            complete =  EnetExtPhy_nwayWait(hPhy);
            if(complete)
            {
                break;
            }
            ClockP_usleep(1000);
        }

        if (complete)
        {
            for (i = 0U; i < timeoutMs; i++)
            {
                isLinkUp = EnetExtPhy_isLinked(hPhy);
                if(isLinkUp)
                {
                    break;
                }
                ClockP_usleep(1000);
            }
        }
    }

    return isLinkUp;
}

EnetExtPhy_Handle EnetExtPhy_open(const EnetExtPhy_Cfg *phyCfg,
                            EnetExtPhy_Mii mii,
                            const EnetExtPhy_LinkCfg *linkCfg,
                            uint32_t macPortCaps,
                            EnetExtPhy_MdioHandle hMdio,
                            void *mdioArgs)
{
    EnetExtPhy_Handle hPhy = EnetExtPhy_getHandle();
    bool alive;
    int32_t status = ENETEXTPHY_SOK;

    if (hPhy == NULL)
    {
        DebugP_logError("PHY %u: Failed to allocate PHY object\r\n", phyCfg->phyAddr);
        status = ENETEXTPHY_EALLOC;
    }

    if (status == ENETEXTPHY_SOK)
    {
        hPhy->hDrv    = NULL;
        hPhy->hMdio   = hMdio;
        hPhy->mdioArgs = mdioArgs;
        hPhy->macCaps = macPortCaps;
        hPhy->phyCfg  = *phyCfg;
        hPhy->mii     = mii;
        hPhy->linkCfg = *linkCfg;
        hPhy->addr    = phyCfg->phyAddr;
        hPhy->group   = phyCfg->phyGroup;

        /* State's linkCaps will be a mask for NWAY path and single bit for manual settings */
        DebugP_assert ((linkCfg->speed == ENETEXTPHY_SPEED_AUTO) ||
                       (linkCfg->duplexity == ENETEXTPHY_DUPLEX_AUTO));
        hPhy->reqLinkCaps = phyCfg->nwayCaps;
        if (hPhy->reqLinkCaps == 0U)
        {
            hPhy->reqLinkCaps = ENETEXTPHY_LINK_CAP_ALL;
        }

        /* Take a shortcut to FOUND state if PHY is already alive, will save some
         * hundreds of msecs.  Take a shorter shortcut to LINK_WAIT state if PHY
         * is strap configured */
        alive = EnetExtPhy_isAlive(hPhy);
        DebugP_assert(alive == true);
        EnetExtPhy_initState(hPhy);

        status = EnetExtPhy_bindDriver(hPhy);

        DebugP_assert(ENETEXTPHY_SOK == status);

        EnetExtPhy_triggerReset(hPhy);

        EnetExtPhy_enable(hPhy);
        DebugP_assert((hPhy->state.isNwayCapable && hPhy->state.enableNway));

        ENETEXTPHYTRACE_DBG_IF((status == ENETEXTPHY_SOK), "PHY %u: open\r\n", hPhy->addr);

        /* Release allocated object in case of any errors */
        if (status != ENETEXTPHY_SOK)
        {
            EnetExtPhy_releaseHandle(hPhy);
        }
    }

    return (status == ENETEXTPHY_SOK) ? hPhy : NULL;
}

void EnetExtPhy_close(EnetExtPhy_Handle hPhy)
{
    /* Power down and isolate the PHY if not strapped */
    DebugP_logInfo("PHY %u: disable\r\n", hPhy->addr);
    EnetExtPhy_rmwReg(hPhy, PHY_BMCR,
                   BMCR_ISOLATE | BMCR_PWRDOWN,
                   BMCR_ISOLATE | BMCR_PWRDOWN);
    EnetExtPhy_releaseHandle(hPhy);
}

bool EnetExtPhy_isAlive(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    bool isAlive = false;
    uint16_t val = 0U;
    int32_t status;

    /* Get PHY alive status */
    if (hMdio->isAlive != NULL)
    {
        /* Get alive status from MDIO driver (i.e. hardware assisted) */
        status = hMdio->isAlive(phyAddr, &isAlive, hPhy->mdioArgs);
        if (status != ENETEXTPHY_SOK)
        {
            DebugP_logError("PHY %u: Failed to get alive status: %d\r\n", phyAddr, status);
        }
    }
    else
    {
        /* Alternatively, read BMSR - PHY is alive if transaction is successful */
        status = hMdio->readC22(phyGroup, phyAddr, PHY_BMSR, &val, hPhy->mdioArgs);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, PHY_BMSR, status);
        if (status == ENETEXTPHY_SOK)
        {
            isAlive = true;
        }
    }

    return isAlive;
}

int32_t EnetExtPhy_getId(EnetExtPhy_Handle hPhy,
                      EnetExtPhy_Version *version)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t id1, id2;
    int32_t status = ENETEXTPHY_SOK;
    bool alive;

    ENETEXTPHYTRACE_VAR(phyAddr);
    alive = EnetExtPhy_isAlive(hPhy);
    if (alive == false)
    {
        status = ENETEXTPHY_EFAIL;
    }

    if (status == ENETEXTPHY_SOK)
    {
        status = EnetExtPhy_readReg(hPhy, PHY_PHYIDR1, &id1);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to read ID1 register: %d\r\n", phyAddr, status);
    }

    if (status == ENETEXTPHY_SOK)
    {
        status = EnetExtPhy_readReg(hPhy, PHY_PHYIDR2, &id2);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to read ID2 register: %d\r\n", phyAddr, status);
    }

    if (status == ENETEXTPHY_SOK)
    {
        version->oui      = ((uint32_t)id1 << PHYIDR1_OUI_OFFSET);
        version->oui     |= (id2 & PHYIDR2_OUI_MASK)  >> PHYIDR2_OUI_OFFSET;
        version->model    = (id2 & PHYIDR2_VMDL_MASK) >> PHYIDR2_VMDL_OFFSET;
        version->revision = (id2 & PHYIDR2_VREV_MASK) >> PHYIDR2_VREV_OFFSET;
    }

    return status;
}

bool EnetExtPhy_isLinked(EnetExtPhy_Handle hPhy)
{
    bool isLinked = EnetExtPhy_isPhyLinked(hPhy);

    return isLinked;
}

int32_t EnetExtPhy_getLinkCfg(EnetExtPhy_Handle hPhy,
                           EnetExtPhy_LinkCfg *linkCfg)
{
    EnetExtPhy_State *state = &hPhy->state;
    bool isLinked;
    int32_t status = ENETEXTPHY_SOK;

    isLinked = EnetExtPhy_isLinked(hPhy);
    if (isLinked)
    {
        linkCfg->speed = state->speed;
        linkCfg->duplexity = state->duplexity;
    }
    else
    {
        ENETEXTPHYTRACE_WARN("PHY %u: PHY is not linked, can't get link config\r\n", hPhy->addr);
        status = ENETEXTPHY_EPERM;
    }

    return status;
}

int32_t EnetExtPhy_readReg(EnetExtPhy_Handle hPhy,
                        uint32_t reg,
                        uint16_t *val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->readC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: reg %u val 0x%04x %s\r\n", phyAddr, reg, *val);

    return status;
}

int32_t EnetExtPhy_writeReg(EnetExtPhy_Handle hPhy,
                         uint32_t reg,
                         uint16_t val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->writeC22(phyGroup, phyAddr, reg, val, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to write reg %u: %d\r\n", phyAddr, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: reg %u val 0x%04x\r\n", phyAddr, reg, val);

    return status;
}

int32_t EnetExtPhy_rmwReg(EnetExtPhy_Handle hPhy,
                       uint32_t reg,
                       uint16_t mask,
                       uint16_t val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;
    uint16_t data = 0U;

    status = hMdio->readC22(phyGroup, phyAddr, reg, &data, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: read reg %u val 0x%04x\r\n", phyAddr, reg, data);

    if (status == ENETEXTPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);

        status = hMdio->writeC22(phyGroup, phyAddr, reg, data, hPhy->mdioArgs);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to write reg %u: %d\r\n", phyAddr, reg, status);
        ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                             "PHY %u: write reg %u val 0x%04x\r\n", phyAddr, reg, data);
    }

    return status;
}

int32_t EnetExtPhy_readExtReg(EnetExtPhy_Handle hPhy,
                           uint32_t reg,
                           uint16_t *val)
{
    int32_t status = ENETEXTPHY_ENOTSUPPORTED;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->readExtReg != NULL))
    {
        status = hPhy->hDrv->readExtReg(hPhy, reg, val);
    }

    return status;
}

int32_t EnetExtPhy_writeExtReg(EnetExtPhy_Handle hPhy,
                            uint32_t reg,
                            uint16_t val)
{
    int32_t status = ENETEXTPHY_ENOTSUPPORTED;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->writeExtReg != NULL))
    {
        status = hPhy->hDrv->writeExtReg(hPhy, reg, val);
    }

    return status;
}

int32_t EnetExtPhy_rmwExtReg(EnetExtPhy_Handle hPhy,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val)
{
    int32_t status = ENETEXTPHY_ENOTSUPPORTED;
    uint16_t data = 0U;

    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->readExtReg != NULL) &&
        (hPhy->hDrv->writeExtReg != NULL))
    {
        status = hPhy->hDrv->readExtReg(hPhy, reg, &data);

        if (status == ENETEXTPHY_SOK)
        {
            data = (data & ~mask) | (val & mask);

            status = hPhy->hDrv->writeExtReg(hPhy, reg, data);
        }
    }

    return status;
}

int32_t EnetExtPhy_readC45Reg(EnetExtPhy_Handle hPhy,
                           uint8_t mmd,
                           uint32_t reg,
                           uint16_t *val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->readC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to read MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: MMD %u reg %u val 0x%04x %s\r\n", phyAddr, mmd, reg, *val);

    return status;
}

int32_t EnetExtPhy_writeC45Reg(EnetExtPhy_Handle hPhy,
                            uint8_t mmd,
                            uint32_t reg,
                            uint16_t val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;

    status = hMdio->writeC45(phyGroup, phyAddr, mmd, reg, val, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to write MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, val);

    return status;
}

int32_t EnetExtPhy_rmwC45Reg(EnetExtPhy_Handle hPhy,
                          uint8_t mmd,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    int32_t status;
    uint16_t data = 0U;

    status = hMdio->readC45(phyGroup, phyAddr, mmd, reg, &data, hPhy->mdioArgs);
    ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                     "PHY %u: Failed to read MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
    ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                         "PHY %u: read MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, data);

    if (status == ENETEXTPHY_SOK)
    {
        data = (data & ~mask) | (val & mask);

        status = hMdio->writeC45(phyGroup, phyAddr, mmd, reg, data, hPhy->mdioArgs);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to write MMD %u reg %u: %d\r\n", phyAddr, mmd, reg, status);
        ENETEXTPHYTRACE_VERBOSE_IF(status == ENETEXTPHY_SOK,
                             "PHY %u: write MMD %u reg %u val 0x%04x\r\n", phyAddr, mmd, reg, data);
    }

    return status;
}

void EnetExtPhy_printRegs(EnetExtPhy_Handle hPhy)
{
    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->printRegs != NULL))
    {
        hPhy->hDrv->printRegs(hPhy);
    }
}

#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_WARN)
static const char *EnetExtPhy_getCapsString(uint32_t linkCaps)
{
    snprintf(gEnetExtPhyCapsBuf, ENETEXTPHY_CAPS_BUF_LEN, "%s%s%s%s%s%s%s",
             ((linkCaps & ENETEXTPHY_LINK_CAP_FD1000) != 0U) ? "FD1000 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_HD1000) != 0U) ? "HD1000 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_FD100) != 0U) ? "FD100 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_HD100) != 0U) ? "HD100 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_FD10) != 0U) ? "FD10 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_HD10) != 0U) ? "HD10 " : "",
             ((linkCaps & ENETEXTPHY_LINK_CAP_ALL) == 0U) ? "None" : "");

    return gEnetExtPhyCapsBuf;
}
#endif

#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG)
static const char *EnetExtPhy_getModeString(EnetExtPhy_Speed speed,
                                         EnetExtPhy_Duplexity duplexity)
{
    snprintf(gEnetExtPhyModeBuf, ENETEXTPHY_MODE_BUF_LEN, "%sbps %s-duplex",
             (speed == ENETEXTPHY_SPEED_1GBIT) ? "1 G" :
             (speed == ENETEXTPHY_SPEED_100MBIT) ? "100 M" : "10 M",
             (duplexity == ENETEXTPHY_DUPLEX_FULL) ? "full" : "half");

    return gEnetExtPhyModeBuf;
}
#endif

static EnetExtPhy_Handle EnetExtPhy_getHandle(void)
{
    EnetExtPhy_Handle hPhy = NULL;
    uint32_t i;

    for (i = 0U; i < ENETEXTPHY_ARRAYSIZE(gEnetExtPhy_phyObjs); i++)
    {
        if (gEnetExtPhy_phyObjs[i].magic != ENETEXTPHY_MAGIC)
        {
            hPhy = &gEnetExtPhy_phyObjs[i];
            memset(hPhy, 0, sizeof(*hPhy));
            hPhy->magic = ENETEXTPHY_MAGIC;
            break;
        }
    }

    return hPhy;
}

static void EnetExtPhy_releaseHandle(EnetExtPhy_Handle hPhy)
{
    hPhy->magic = ENETEXTPHY_NO_MAGIC;
}


static void EnetExtPhy_initState(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;

    state->speed = ENETEXTPHY_SPEED_10MBIT;
    state->duplexity = ENETEXTPHY_DUPLEX_HALF;
    state->phyLinkCaps = 0U;

    state->needsManualCfg = true;
    state->needsNwayCfg   = true;
}

static void EnetExtPhy_enable(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;
    uint32_t socLinkCaps;
    uint32_t commonLinkCaps;

    /* Enable PHY (resetting it if applicable) can be done only when
     * entering the ENABLE state for the first time */
    if (state->needsNwayCfg ||
        state->needsManualCfg)
    {
        /* Set PHY in normal mode */
        DebugP_logInfo("PHY %u: enable\r\n", hPhy->addr);
        EnetExtPhy_rmwReg(hPhy, PHY_BMCR, BMCR_ISOLATE | BMCR_PWRDOWN, 0U);

        /* PHY-specific 'extended' configuration */
        if ((hPhy->hDrv->config != NULL) &&
            !hPhy->phyCfg.skipExtendedCfg)
        {
            hPhy->hDrv->config(hPhy, &hPhy->phyCfg, hPhy->mii);
        }
    }

    /* Find common capabilities among app's, PHY's and SoC's */
    DebugP_logInfo("PHY %u: req caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(hPhy->reqLinkCaps));

    state->phyLinkCaps = EnetExtPhy_getLocalCaps(hPhy);
    DebugP_logInfo("PHY %u: PHY caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(state->phyLinkCaps));

    socLinkCaps = hPhy->macCaps;
    DebugP_logInfo("PHY %u: MAC caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(socLinkCaps));

    commonLinkCaps = hPhy->reqLinkCaps &
                     state->phyLinkCaps &
                     socLinkCaps;
    DebugP_logInfo("PHY %u: refined caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(commonLinkCaps));

    if (commonLinkCaps != 0U)
    {
        state->linkCaps = commonLinkCaps;

        /* Enable NWAY if requested by app (via auto-speed or auto-duplex) and
         * supported by PHY */
        state->isNwayCapable = EnetExtPhy_isNwayCapable(hPhy);
        DebugP_logInfo("PHY %u: PHY is %sNWAY-capable\r\n",
                      hPhy->addr, state->isNwayCapable ? "" : "not ");

        state->enableNway = (hPhy->linkCfg.speed == ENETEXTPHY_SPEED_AUTO) ||
                            (hPhy->linkCfg.duplexity == ENETEXTPHY_DUPLEX_AUTO);
        if (state->enableNway && !state->isNwayCapable)
        {
            /* If PHY is not capable of auto negotiation, fallback to manual mode
             * with the highest refined capability */
            ENETEXTPHYTRACE_WARN("PHY %u: falling back to manual mode\r\n", hPhy->addr);
            state->enableNway = false;

            commonLinkCaps = EnetExtPhy_findBestCap(commonLinkCaps);
            ENETEXTPHYTRACE_WARN("PHY %u: new link caps: %s\r\n",
                           hPhy->addr, EnetExtPhy_getCapsString(commonLinkCaps));
            state->linkCaps = commonLinkCaps;
        }
    }

    DebugP_assert((commonLinkCaps != 0U) && (state->isNwayCapable && state->enableNway));
    DebugP_logInfo("PHY %u: setup NWAY\r\n", hPhy->addr);
    EnetExtPhy_setupNway(hPhy);
}

static void EnetExtPhy_setupNway(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;
    uint16_t nwayAdvertise;
    uint16_t nway1000Advertise = 0U;

    DebugP_logInfo("PHY %u: NWAY advertising: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(state->linkCaps));

    DebugP_logInfo("PHY %u: config is %sneeded\r\n",
                  hPhy->addr, state->needsNwayCfg ? "" : "not ");

    if (state->needsNwayCfg)
    {
        nwayAdvertise = ANAR_802P3;
        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_FD100) != 0U)
        {
            nwayAdvertise |= ANAR_100FD;
        }

        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_HD100) != 0U)
        {
            nwayAdvertise |= ANAR_100HD;
        }

        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_FD10) != 0U)
        {
            nwayAdvertise |= ANAR_10FD;
        }

        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_HD10) != 0U)
        {
            nwayAdvertise |= ANAR_10HD;
        }

        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_FD1000) != 0U)
        {
            nway1000Advertise |= GIGCR_1000FD;
        }

        if ((state->linkCaps & ENETEXTPHY_LINK_CAP_HD1000) != 0U)
        {
            nway1000Advertise |= GIGCR_1000HD;
        }

        EnetExtPhy_rmwReg(hPhy, PHY_ANAR, ANAR_100 | ANAR_10, nwayAdvertise);

        if ((state->phyLinkCaps & ENETEXTPHY_LINK_CAP_1000) != 0U)
        {
            EnetExtPhy_rmwReg(hPhy, PHY_GIGCR, GIGCR_1000, nway1000Advertise);
        }

        state->needsNwayCfg = false;
    }

    /* Restart auto-negotiation */
    DebugP_logInfo("PHY %u: restart autonegotiation\r\n", hPhy->addr);
    EnetExtPhy_rmwReg(hPhy, PHY_BMCR, BMCR_ANEN, BMCR_ANEN);

    /* TODO: is MII_ENETEXTPHY_FD needed for auto-negotiation? */
    EnetExtPhy_rmwReg(hPhy, PHY_BMCR,
                   BMCR_ANRESTART | BMCR_FD,
                   BMCR_ANRESTART | BMCR_FD);
}

static bool EnetExtPhy_nwayStart(EnetExtPhy_Handle hPhy)
{
    uint16_t mode;
    uint16_t val;
    bool  nwayStartDone;

    /* Wait for NWAY to start */
    EnetExtPhy_readReg(hPhy, PHY_BMCR, &mode);

    if ((mode & BMCR_ANRESTART) == 0U)
    {
        /* Flush pending latch bits */
        EnetExtPhy_readReg(hPhy, PHY_BMSR, &val);
        nwayStartDone = true;
    }
    else
    {
        nwayStartDone = false;
    }
    return nwayStartDone;
}

static bool EnetExtPhy_nwayWait(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;
    uint32_t nwayCaps;
    uint16_t status;
    bool nWayWaitDone;

    EnetExtPhy_readReg(hPhy, PHY_BMSR, &status);
    if (0U != (status & BMSR_ANCOMPLETE))
    {
        nwayCaps = EnetExtPhy_findCommonNwayCaps(hPhy);
        if (nwayCaps != 0U)
        {
            EnetExtPhy_capToMode(nwayCaps, &state->speed, &state->duplexity);

            DebugP_logInfo("PHY %u: negotiated mode: %s\r\n",
                          hPhy->addr, EnetExtPhy_getModeString(state->speed, state->duplexity));

        }
        nWayWaitDone = true;
    }
    else
    {
        nWayWaitDone = false;
    }
    return nWayWaitDone;
}

static bool EnetExtPhy_checkLink(EnetExtPhy_Handle hPhy)
{
    uint16_t status;
    bool isLinkUp = false;

    EnetExtPhy_readReg(hPhy, PHY_BMSR, &status);

    if ((status & BMSR_LINKSTS) != 0U)
    {
        isLinkUp = true;
    }
    return isLinkUp;
}

static void EnetExtPhy_handleLinkDown(EnetExtPhy_Handle hPhy)
{
    bool linked = EnetExtPhy_isPhyLinked(hPhy);

    /* Not linked */
    if (!linked)
    {
        EnetExtPhy_triggerReset(hPhy);

        EnetExtPhy_enable(hPhy);
        DebugP_assert((hPhy->state.isNwayCapable && hPhy->state.enableNway));

        do 
        {
            linked = EnetExtPhy_WaitForLinkUp(hPhy, ENETEXTPHY_TIMEOUT_MS);
        } while (!linked);
    }
}



static bool EnetExtPhy_isNwayCapable(EnetExtPhy_Handle hPhy)
{
    uint16_t val = 0U;

    /* Get the PHY Status */
    EnetExtPhy_readReg(hPhy, PHY_BMSR, &val);

    return ((val & BMSR_ANCAPABLE) != 0U);
}

static uint32_t EnetExtPhy_getLocalCaps(EnetExtPhy_Handle hPhy)
{
    uint32_t caps = 0U;
    uint16_t val  = 0U;

    /* Get 10/100 Mbps capabilities */
    EnetExtPhy_readReg(hPhy, PHY_BMSR, &val);
    if ((val & BMSR_100FD) != 0U)
    {
        caps |= ENETEXTPHY_LINK_CAP_FD100;
    }

    if ((val & BMSR_100HD) != 0U)
    {
        caps |= ENETEXTPHY_LINK_CAP_HD100;
    }

    if ((val & BMSR_10FD) != 0U)
    {
        caps |= ENETEXTPHY_LINK_CAP_FD10;
    }

    if ((val & BMSR_10HD) != 0U)
    {
        caps |= ENETEXTPHY_LINK_CAP_HD10;
    }

    /* Get extended (1 Gbps) capabilities if supported */
    if ((val & BMSR_GIGEXTSTS) != 0U)
    {
        EnetExtPhy_readReg(hPhy, PHY_GIGESR, &val);

        if ((val & GIGESR_1000FD) != 0U)
        {
            caps |= ENETEXTPHY_LINK_CAP_FD1000;
        }

        if ((val & GIGESR_1000HD) != 0U)
        {
            caps |= ENETEXTPHY_LINK_CAP_HD1000;
        }
    }

    return caps;
}

static uint32_t EnetExtPhy_findCommonCaps(EnetExtPhy_Handle hPhy)
{
    uint32_t localCaps = 0U;
    uint32_t partnerCaps = 0U;
    uint16_t val = 0U;

    /* Get local device capabilities */
    EnetExtPhy_readReg(hPhy, PHY_ANAR, &val);

    if ((val & ANAR_100FD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_FD100;
    }

    if ((val & ANAR_100HD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_HD100;
    }

    if ((val & ANAR_10FD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_FD10;
    }

    if ((val & ANAR_10HD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_HD10;
    }

    /* Get link partner capabilities */
    val = 0U;
    EnetExtPhy_readReg(hPhy, PHY_ANLPAR, &val);

    if ((val & ANLPAR_100FD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_FD100;
    }

    if ((val & ANLPAR_100HD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_HD100;
    }

    if ((val & ANLPAR_10FD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_FD10;
    }

    if ((val & ANLPAR_10HD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_HD10;
    }

    DebugP_logInfo("PHY %u: local caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(localCaps));
    DebugP_logInfo("PHY %u: partner caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(partnerCaps));
    DebugP_logInfo("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(localCaps & partnerCaps));

    return (localCaps & partnerCaps);
}

static uint32_t EnetExtPhy_findCommon1000Caps(EnetExtPhy_Handle hPhy)
{
    uint32_t localCaps = 0U;
    uint32_t partnerCaps = 0U;
    uint16_t val;

    /* Get local device capabilities */
    EnetExtPhy_readReg(hPhy, PHY_GIGCR, &val);

    if ((val & GIGCR_1000FD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_FD1000;
    }

    if ((val & GIGCR_1000HD) != 0U)
    {
        localCaps |= ENETEXTPHY_LINK_CAP_HD1000;
    }

    /* Get link partner capabilities */
    EnetExtPhy_readReg(hPhy, PHY_GIGSR, &val);

    if ((val & GIGSR_1000FD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_FD1000;
    }

    if ((val & GIGSR_1000FD) != 0U)
    {
        partnerCaps |= ENETEXTPHY_LINK_CAP_HD1000;
    }

    DebugP_logInfo("PHY %u: local caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(localCaps));
    DebugP_logInfo("PHY %u: partner caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(partnerCaps));
    DebugP_logInfo("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(localCaps & partnerCaps));

    return (localCaps & partnerCaps);
}

static uint32_t EnetExtPhy_findCommonNwayCaps(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;
    uint32_t nwayCaps;

    nwayCaps = EnetExtPhy_findCommonCaps(hPhy);

    /* Find common gigabit capabilities if supported by PHY */
    if ((state->phyLinkCaps & ENETEXTPHY_LINK_CAP_1000) != 0U)
    {
        nwayCaps |= EnetExtPhy_findCommon1000Caps(hPhy);
    }

    DebugP_logInfo("PHY %u: common caps: %s\r\n",
                  hPhy->addr, EnetExtPhy_getCapsString(nwayCaps));

    if (nwayCaps != 0U)
    {
        /* Find the highest performance protocol from the common
         * capabilities of local device and link partner */
        nwayCaps = EnetExtPhy_findBestCap(nwayCaps);
    }
    else
    {
        /* Non-compiant PHY? Flag the negotiation error */
        ENETEXTPHYTRACE_ERR("PHY %u: no common caps found\r\n", hPhy->addr);
    }

    return nwayCaps;
}

static bool EnetExtPhy_isPhyLinked(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_MdioHandle hMdio = hPhy->hMdio;
    uint32_t phyGroup = hPhy->group;
    uint32_t phyAddr = hPhy->addr;
    bool isLinked = false;
    uint16_t val = 0U;
    int32_t status;

    /* Get PHY link status */
    if (hMdio->isLinked != NULL)
    {
        /* Get link status from MDIO driver (i.e. hardware assisted) */
        status = hMdio->isLinked(phyAddr, &isLinked, hPhy->mdioArgs);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to get link status: %d\r\n", phyAddr, status);
    }
    else
    {
        /* Alternatively, BMSR[2] Link Status bit can be checked */
        status = hMdio->readC22(phyGroup, phyAddr, PHY_BMSR, &val, hPhy->mdioArgs);
        ENETEXTPHYTRACE_ERR_IF(status != ENETEXTPHY_SOK,
                         "PHY %u: Failed to read reg %u: %d\r\n", phyAddr, PHY_BMSR, status);
        if ((status == ENETEXTPHY_SOK) &&
            ((val & BMSR_LINKSTS) != 0U))
        {
            isLinked = true;
        }
    }

    return isLinked;
}

static int32_t EnetExtPhy_bindDriver(EnetExtPhy_Handle hPhy)
{
    EnetExtPhyDrv_Handle hDrv;
    EnetExtPhy_Version version;
    bool match;
    bool macSupported;
    uint32_t i;
    int32_t status;

    hPhy->hDrv = NULL;

    status = EnetExtPhy_getId(hPhy, &version);
    if (status == ENETEXTPHY_SOK)
    {
        for (i = 0U; i < ENETEXTPHY_ARRAYSIZE(gEnetExtPhyDrvs); i++)
        {
            hDrv = gEnetExtPhyDrvs[i];

            DebugP_assert(hDrv != NULL);
            DebugP_assert(hDrv->isPhyDevSupported != NULL);

            DebugP_logInfo("PHY %u: OUI:%06x Model:%02x Ver:%02x <-> '%s'\r\n",
                          hPhy->addr,
                          version.oui, version.model, version.revision,
                          hDrv->name);

            /* Check if driver supports detected PHY device */
            match = hDrv->isPhyDevSupported(hPhy, &version);

            /* Check if driver supports the MAC mode */
            if (match)
            {
                macSupported = hDrv->isMacModeSupported(hPhy, hPhy->mii);
                if (!macSupported)
                {
                    ENETEXTPHYTRACE_WARN("PHY %u: '%s' doesn't support MAC mode %d\r\n",
                                   hPhy->addr, hDrv->name, hPhy->mii);
                }
            }

            /* Bind the device <-> driver now */
            if (match)
            {
                hPhy->hDrv = hDrv;
                break;
            }
        }

        ENETEXTPHYTRACE_INFO_IF(hPhy->hDrv != NULL,
                          "PHY %u: OUI:%06x Model:%02x Ver:%02x <-> '%s' : OK\r\n",
                          hPhy->addr,
                          version.oui, version.model, version.revision,
                          hDrv->name);
    }

    return (hPhy->hDrv != NULL) ? ENETEXTPHY_SOK : ENETEXTPHY_EFAIL;
}

static void EnetExtPhy_resetPhy(EnetExtPhy_Handle hPhy)
{
    EnetExtPhy_State *state = &hPhy->state;

    ENETEXTPHYTRACE_WARN_IF(hPhy->hDrv == NULL,
                      "PHY %u: not bound to a driver, can't be reset\r\n", hPhy->addr);

    /* PHY specific reset */
    if ((hPhy->hDrv != NULL) &&
        (hPhy->hDrv->reset != NULL))
    {
        hPhy->hDrv->reset(hPhy);

        /* Typically manual/auto-negotiation should be reconfigured after PHY reset.
         * Certain reset/restart implementations only reset the logic not registers,
         * but it's safer to assume reconfiguration is needed */
        state->needsManualCfg = true;
        state->needsNwayCfg   = true;
    }
}

static uint32_t EnetExtPhy_findBestCap(uint32_t caps)
{
    uint32_t i;

    for (i = ENETEXTPHY_LINK_CAP_FD1000; i >= ENETEXTPHY_LINK_CAP_HD10; i >>= 1U)
    {
        if ((caps & i) != 0U)
        {
            break;
        }
    }

    return i;
}

static void EnetExtPhy_capToMode(uint32_t caps,
                              EnetExtPhy_Speed *speed,
                              EnetExtPhy_Duplexity *duplexity)
{
    if (caps & ENETEXTPHY_LINK_CAP_FD1000)
    {
        *speed  = ENETEXTPHY_SPEED_1GBIT;
        *duplexity = ENETEXTPHY_DUPLEX_FULL;
    }
    else if (caps & ENETEXTPHY_LINK_CAP_HD1000)
    {
        *speed  = ENETEXTPHY_SPEED_1GBIT;
        *duplexity = ENETEXTPHY_DUPLEX_HALF;
    }
    else if (caps & ENETEXTPHY_LINK_CAP_FD100)
    {
        *speed  = ENETEXTPHY_SPEED_100MBIT;
        *duplexity = ENETEXTPHY_DUPLEX_FULL;
    }
    else if (caps & ENETEXTPHY_LINK_CAP_HD100)
    {
        *speed  = ENETEXTPHY_SPEED_100MBIT;
        *duplexity = ENETEXTPHY_DUPLEX_HALF;
    }
    else if (caps & ENETEXTPHY_LINK_CAP_FD10)
    {
        *speed  = ENETEXTPHY_SPEED_10MBIT;
        *duplexity = ENETEXTPHY_DUPLEX_FULL;
    }
    else
    {
        *speed  = ENETEXTPHY_SPEED_10MBIT;
        *duplexity = ENETEXTPHY_DUPLEX_HALF;
    }
}

static void EnetExtPhy_showLinkPartnerCompat(EnetExtPhy_Handle hPhy,
                                          EnetExtPhy_Speed speed,
                                          EnetExtPhy_Duplexity duplexity)
{
    switch (speed)
    {
        case ENETEXTPHY_SPEED_10MBIT:
        case ENETEXTPHY_SPEED_100MBIT:
            if (duplexity == ENETEXTPHY_DUPLEX_HALF)
            {
                DebugP_logInfo("PHY %u: recommended link partner config: "
                              "manual mode at %s or auto-negotiation\r\n",
                              hPhy->addr, EnetExtPhy_getModeString(speed, duplexity));
            }
            else
            {
                DebugP_logInfo("PHY %u: recommended link partner config: "
                              "manual mode at %s\r\n",
                              hPhy->addr, EnetExtPhy_getModeString(speed, duplexity));
            }

            break;

        case ENETEXTPHY_SPEED_1GBIT:
            DebugP_logInfo("PHY %u: recommended link partner config: "
                          "manual mode at %s\r\n",
                          hPhy->addr, EnetExtPhy_getModeString(speed, duplexity));
            break;

        default:
            break;
    }
}
