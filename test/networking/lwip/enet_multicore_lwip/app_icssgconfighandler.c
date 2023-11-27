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
 * app_icssgconfighandler.c - This file is part of lwIP test
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
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>
#include <include/per/icssg.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
//static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
//                                         void *appArg);
#endif

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);

static bool EnetMp_isRgmiiEnabled (Enet_Type enetType, uint32_t instId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define MII_LINK0_EVENT      41
#define MII_LINK1_EVENT      53

extern EnetAppInstInfo gInstInfo[ENET_SYSCFG_NUM_PERIPHERAL];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static bool EnetMp_isRgmiiEnabled (Enet_Type enetType, uint32_t instId)
{
    uint32_t i;
    for (i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
    {
        if ((enetType == gInstInfo[i].enetType) && (instId == gInstInfo[i].instId))
        {
            break;
        }
    }
    return gInstInfo[i].rgmiiEn;
}

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
    bool isRgmiiEn = EnetMp_isRgmiiEnabled(enetType, instId);

    /* Setup board for requested Ethernet port */
    ethPort.enetType = enetType;
    ethPort.instId   = instId;
    ethPort.macPort  = macPort;
    if (!isRgmiiEn)
    {
        ethPort.boardId  = ENETBOARD_MII_ID;
        ethPort.mii.layerType    = ENET_MAC_LAYER_MII;
        ethPort.mii.sublayerType = ENET_MAC_SUBLAYER_STANDARD;
        ethPort.mii.variantType  = ENET_MAC_VARIANT_NONE;
    }
    else
    {
        ethPort.boardId  = ENETBOARD_AM64X_AM243X_EVM;
        ethPort.mii.layerType      = ENET_MAC_LAYER_GMII;
        ethPort.mii.sublayerType   = ENET_MAC_SUBLAYER_REDUCED;
        ethPort.mii.variantType    = ENET_MAC_VARIANT_FORCED;
    }

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    {
	    IcssgMacPort_Cfg *macCfg = (IcssgMacPort_Cfg *)linkArgs->macCfg;
	    IcssgMacPort_initCfg(macCfg);
	    macCfg->specialFramePrio = 1U;
    }

    /* Set port link params */
    linkArgs->macPort = macPort;

    mii->layerType     = ethPort.mii.layerType;
    mii->sublayerType  = ethPort.mii.sublayerType;

    if (!isRgmiiEn)
    {
        mii->variantType   = ethPort.mii.variantType;
    }
    else
    {
        mii->variantType   = ENET_MAC_VARIANT_FORCED;
    }
    linkCfg->speed     = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

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
}

static void EnetApp_updateMdioLinkIntCfg(Enet_Type enetType, uint32_t instId, Icssg_mdioLinkIntCfg *mdioLinkIntCfg)
{
    /*! INTC Module mapping data passed by application for configuring PRU to R5F interrupts */
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss0_intc_initdata;
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss1_intc_initdata;
#endif
    mdioLinkIntCfg->coreIntrNum = 254;
    mdioLinkIntCfg->pruEvtNum[0] = MII_LINK0_EVENT;
    mdioLinkIntCfg->pruEvtNum[1] = MII_LINK1_EVENT;
    mdioLinkIntCfg->isPulseIntr = 0;
    mdioLinkIntCfg->intrPrio = 15;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = NULL;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
#else
    #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
        EnetApp_initMdioLinkIntCfg(enetType, instId, icssgCfg);
    #else
//        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = &EnetApp_mdioLinkStatusChange;
        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
    #endif
    EnetApp_updateMdioLinkIntCfg(enetType, instId, &icssgCfg->mdioLinkIntCfg);
#endif
    EnetRm_ResCfg *resCfg;
//    uint32_t i;
//    uint32_t perIdx;
//    int32_t status;
    bool isRgmiiEn = EnetMp_isRgmiiEnabled(enetType, instId);

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit  configs EnetType:%u, InstId :%u\r\n", enetType, instId);
    EnetAppUtils_print("----------------------------------------------\r\n");

    if (!isRgmiiEn)
    {
        icssgCfg->mii.layerType    = ENET_MAC_LAYER_MII;
        icssgCfg->mii.sublayerType = ENET_MAC_SUBLAYER_STANDARD;
        icssgCfg->mii.variantType  = ENET_MAC_VARIANT_NONE;
    }

    resCfg = &icssgCfg->resCfg;

    /* We use software MAC address pool from apputils, but it will give same MAC address.
        * Add port index to make them unique */
//    status = EnetMp_getPerIdx(enetType, instId, &perIdx);
//    EnetAppUtils_assert(status == ENET_SOK);
//    for (i = 0U; i < ENETMP_PORT_MAX; i++)
//    {
//        resCfg->macList.macAddress[i][ENET_MAC_ADDR_LEN - 1] += (perIdx * ENETMP_PORT_MAX);
//    }
    resCfg->macList.numMacAddress = ENET_SYSCFG_MAX_MAC_PORTS;
}

#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
//static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
//                                         void *appArg)
//{
//    static uint32_t linkUpCount = 0;
//    if ((info->linkChanged) && (info->isLinked))
//    {
//        linkUpCount++;
//    }
//}
#endif

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}
