/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * app_cpswconfighandler.c - This file is part of lwIP test
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
/* SDK includes */
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include <enet.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg);

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    int32_t status;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = enetType;
    ethPort.instId   = instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = EnetBoard_getId();

    /* Get the Mii config for Ethernet port */
    EnetBoard_getMiiConfig(&ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    if (Enet_isCpswFamily(ethPort.enetType))
    {
        CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)linkArgs->macCfg;
        CpswMacPort_initCfg(macCfg);
        if (EnetMacPort_isSgmii(mii) || EnetMacPort_isQsgmii(mii))
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_SGMII_WITH_PHY;
        }
        else
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_INVALID;
        }
    }

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL)
    {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr     = boardPhyCfg->phyAddr;
        phyCfg->isStrapped  = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn  = false;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);

    }
    else
    {
        DebugP_log("No PHY configuration found for MAC port %u\r\n",
                           ENET_MACPORT_ID(ethPort.macPort));
        EnetAppUtils_assert(false);
    }

    mii->layerType     = ethPort.mii.layerType;
    mii->sublayerType  = ethPort.mii.sublayerType;
    mii->variantType   = ENET_MAC_VARIANT_FORCED;
    linkCfg->speed     = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    if (Enet_isCpswFamily(ENET_CPSW_3G))
    {
        CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)linkArgs->macCfg;

        if (EnetMacPort_isSgmii(mii) || EnetMacPort_isQsgmii(mii))
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_SGMII_WITH_PHY;
        }
        else
        {
            macCfg->sgmiiMode = ENET_MAC_SGMIIMODE_INVALID;
        }
    }
}


void EnetApp_addMCastEntry(Enet_Type enetType,
                          uint32_t instId,
                          uint32_t coreId,
                          const uint8_t *testMCastAddr,
                          uint32_t portMask)
{
    Enet_IoctlPrms prms;
    int32_t status;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;

    if (Enet_isCpswFamily(enetType))
    {
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);

        EnetAppUtils_assert(hEnet != NULL);
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
               sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                               status);
        }
    }
}


void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetCpdma_Cfg * dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);
    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
#endif


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb    = &EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif
    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;

#if defined (ENET_CPSW_ENABLE_DUAL_MAC)
    /* Enabling MAC only mode for CPSW*/
    uint32_t i;
    for (i = 0U; i < ENET_ARRAYSIZE(cpswCfg->aleCfg.portCfg); i++)
    {
        cpswCfg->aleCfg.portCfg[i].macModeCfg.macOnlyEn = true;
    }
#endif
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}
