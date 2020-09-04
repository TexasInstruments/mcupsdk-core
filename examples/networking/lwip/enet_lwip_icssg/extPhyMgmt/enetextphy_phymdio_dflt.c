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
 * \file  enetextphy_phymdio_dflt.c
 *
 * \brief This file contains the default implementation of the MDIO interface
 *        of the Ethernet PHY (ENETEXTPHY) driver with Enet LLD APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enetextphy.h"
#include "enetextphy_priv.h"
#include <enet.h>
#include <enet_cfg.h>
#include <networking/enet/core/include/mod/mdio.h>
#include <networking/enet/core/include/core/enet_soc.h>
#include "ti_enet_config.h"

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

static int32_t EnetExtPhyMdioDflt_isAlive(uint32_t phyAddr,
                                bool *isAlive,
                                void *args);

static int32_t EnetExtPhyMdioDflt_isLinked(uint32_t phyAddr,
                                 bool *isLinked,
                                 void *args);

static int32_t EnetExtPhyMdioDflt_readC22(uint32_t group,
                                uint32_t phyAddr,
                                uint32_t reg,
                                uint16_t *val,
                                void *args);

static int32_t EnetExtPhyMdioDflt_writeC22(uint32_t group,
                                 uint32_t phyAddr,
                                 uint32_t reg,
                                 uint16_t val,
                                 void *args);

static int32_t EnetExtPhyMdioDflt_readC45(uint32_t group,
                                uint32_t phyAddr,
                                uint8_t mmd,
                                uint16_t reg,
                                uint16_t *val,
                                void *args);

static int32_t EnetExtPhyMdioDflt_writeC45(uint32_t group,
                                 uint32_t phyAddr,
                                 uint8_t mmd,
                                 uint16_t reg,
                                 uint16_t val,
                                 void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static EnetExtPhy_Mdio gEnetExtPhy_PhyMdioDflt =
{
    .isAlive  = EnetExtPhyMdioDflt_isAlive,
    .isLinked = EnetExtPhyMdioDflt_isLinked,
    .readC22  = EnetExtPhyMdioDflt_readC22,
    .writeC22 = EnetExtPhyMdioDflt_writeC22,
    .readC45  = EnetExtPhyMdioDflt_readC45,
    .writeC45 = EnetExtPhyMdioDflt_writeC45,
};


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

EnetExtPhy_MdioHandle EnetExtPhyMdioDflt_getPhyMdio(void)
{
    return &gEnetExtPhy_PhyMdioDflt;
}

static int32_t EnetExtPhyMdioDflt_isAlive(uint32_t phyAddr,
                                bool *isAlive,
                                void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, isAlive);

        ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_IS_ALIVE, &prms, status);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to get MDIO alive status: %d\n", phyAddr, status);
    }

    return status;
}

static int32_t EnetExtPhyMdioDflt_isLinked(uint32_t phyAddr,
                                 bool *isLinked,
                                 void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, isLinked);

        ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_IS_LINKED, &prms, status);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to get link status: %d\n", phyAddr, status);
    }

    return status;
}

static int32_t EnetExtPhyMdioDflt_readC22(uint32_t group,
                                uint32_t phyAddr,
                                uint32_t reg,
                                uint16_t *val,
                                void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    EnetMdio_C22ReadInArgs inArgs;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.reg = reg;

#if ENET_SYSCFG_ENABLE_MDIO_MANUALMODE
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, val);
        ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_READ, &prms, status);
#else

        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER, &prms, status);
        } while (status != ENET_SOK);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C22 reg: %d\n", phyAddr, status);
        DebugP_assert(status == ENET_SOK);

        do {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, val);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE, &prms, status);
        } while (status == ENET_SINPROGRESS);
#endif
        DebugP_assert(status == ENET_SOK);
    }

    return status;
}

static int32_t EnetExtPhyMdioDflt_writeC22(uint32_t group,
                                 uint32_t phyAddr,
                                 uint32_t reg,
                                 uint16_t val,
                                 void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    EnetMdio_C22WriteInArgs inArgs;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.reg = reg;
        inArgs.val = val;

#if ENET_SYSCFG_ENABLE_MDIO_MANUALMODE
        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_WRITE, &prms, status);
#else

        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER, &prms, status);
        } while (status != ENET_SOK);
        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE, &prms, status);
        } while (status == ENET_SINPROGRESS);
#endif
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to write C22 reg: %d\n", phyAddr, status);
    }

    return status;
}

static int32_t EnetExtPhyMdioDflt_readC45(uint32_t group,
                                uint32_t phyAddr,
                                uint8_t mmd,
                                uint16_t reg,
                                uint16_t *val,
                                void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    EnetMdio_C45ReadInArgs inArgs;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.mmd = (EnetMdio_C45Mmd)mmd;
        inArgs.reg = reg;

        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER, &prms, status);
        } while (status != ENET_SOK);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C22 reg: %d\n", phyAddr, status);
        DebugP_assert(status == ENET_SOK);

        do {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, val);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE, &prms, status);
        } while (status == ENET_SINPROGRESS);

        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C45 reg: %d\n", phyAddr, status);
    }

    return status;
}

static int32_t EnetExtPhyMdioDflt_writeC45(uint32_t group,
                                 uint32_t phyAddr,
                                 uint8_t mmd,
                                 uint16_t reg,
                                 uint16_t val,
                                 void *args)
{
    Enet_Handle hEnet = args;
    Enet_IoctlPrms prms;
    EnetMdio_C45WriteInArgs inArgs;
    int32_t status = ENETEXTPHY_SOK;

    if (hEnet == NULL)
    {
        ENETEXTPHYTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETEXTPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.mmd = (EnetMdio_C45Mmd)mmd;
        inArgs.reg = reg;
        inArgs.val = val;

        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER, &prms, status);
        } while (status != ENET_SOK);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C22 reg: %d\n", phyAddr, status);
        DebugP_assert(status == ENET_SOK);

        do {
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(hEnet, EnetSoc_getCoreId(), ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE, &prms, status);
        } while (status == ENET_SINPROGRESS);
        ENETEXTPHYTRACE_ERR_IF(status != ENET_SOK, "Failed to write C45 reg: %d\n", status);
    }

    return status;
}

