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
 * test.c - This file is part of lwIP test
 *
 */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* lwIP core includes */
#include "lwip/opt.h"
#include "test_enet_lwip.h"
#include "ti_board_config.h"
#include <networking/enet/core/include/per/icssg.h>

/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "enetextphy.h"
#include "enetextphy_phymdio_dflt.h"
#include <include/per/icssg.h>
#include <priv/per/icssg_priv.h>
#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
#include "enetextphy.h"
#include "enetextphy_phymdio_dflt.h"
#include "test_enet_extphy.h"
#include "portmacro.h"
#endif /* ENET_SYSCFG_ENABLE_EXTPHY == 1U */

#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
#define ENETAPP_EXT_PHY_NUM_ENABLED_PORTS       (ENET_SYSCFG_MAX_MAC_PORTS)
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
#define ENETAPP_PHY_REGISTERPOLL_TASK_PRIORITY  (7U)
#define ENETAPP_PHY_REGISTERPOLL_TASK_STACK     (3 * 1024)
#define ENET_PHY_REGISTER_POLLING_PERIOD_MS     (1000U)
#endif /* ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U */
#endif /*ENET_SYSCFG_ENABLE_EXTPHY == 1U */

typedef struct EnetApp_AppEnetInfo
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports List to use for the above EnetType & InstId*/
    uint8_t     numMacPort;

    /* Num MAC ports to use for the above EnetType & InstId*/
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];
} EnetApp_AppEnetInfo;

#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
static void EnetApp_addMCastEntry(Enet_Type enetType,
                                  uint32_t instId,
                                  uint32_t coreId,
                                  const uint8_t *testMCastAddr,
                                  uint32_t portMask);

static void EnetApp_initExtPhy(Enet_Type enetType,
                               uint32_t instId,
                               Enet_MacPort macPort);
static void EnetApp_enableMdioStateMachine(Enet_Type enetType,
                                           uint32_t instId,
                                           uint32_t coreId);
static void EnetApp_waitForPhyAlive(Enet_Type enetType,
                                    uint32_t instId,
                                    uint32_t coreId,
                                    Enet_MacPort macPort);

EnetExtPhy_Handle EnetApp_getExtPhyHandle(uint32_t portIdx);

#endif /* ENET_SYSCFG_ENABLE_EXTPHY == 1U */

static EnetApp_AppEnetInfo gEnetAppParams[ENET_SYSCFG_MAX_ENET_INSTANCES];

void print_cpu_load()
{
    static uint32_t start_time = 0;
    uint32_t print_interval_in_secs = 5;
    uint32_t cur_time = ClockP_getTimeUsec()/1000;

    if(start_time==0)
    {
        start_time = cur_time;
    }
    else
    if( (cur_time-start_time) >= (print_interval_in_secs*1000) )
    {
        uint32_t cpu_load = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%2d %%\r\n",
            cur_time/1000, cur_time%1000,
            cpu_load/100, cpu_load%100 );

        start_time = cur_time;

        TaskP_loadResetAll();
    }
}

int enet_lwip_example(void *args)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("      ENET LWIP App       \r\n");
    DebugP_log("==========================\r\n");

    EnetApp_driverInit();

    /* Read MAC Port details and enable clock for each ENET instance */
    for(i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
    {
        EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0 + i, &gEnetAppParams[i].enetType, &gEnetAppParams[i].instId);
        EnetApp_getEnetInstMacInfo(gEnetAppParams[i].enetType,
                                   gEnetAppParams[i].instId,
                                   gEnetAppParams[i].macPortList,
                                   &gEnetAppParams[i].numMacPort);
        EnetAppUtils_enableClocks(gEnetAppParams[i].enetType, gEnetAppParams[i].instId);
    }

    /* Open ENET driver for each ENET instance */
    for(i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
    {
        status = EnetApp_driverOpen(gEnetAppParams[i].enetType, gEnetAppParams[i].instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET[i]: %d\r\n", i, status);
            EnetAppUtils_assert(status == ENET_SOK);
        }
    }


#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
    {
        uint32_t phyAddMask = 0;
        bool islinked = false;

        Enet_Type enetTypeList[ENET_SYSCFG_MAX_MAC_PORTS];
        uint32_t  instIdList[ENET_SYSCFG_MAX_MAC_PORTS];

        for (i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
        {
            for (uint32_t portIdx = 0; portIdx < gEnetAppParams[i].numMacPort; portIdx++)
            {
                enetTypeList[gEnetAppParams[i].macPortList[portIdx]] = gEnetAppParams[i].enetType;
                instIdList[gEnetAppParams[i].macPortList[portIdx]]   = gEnetAppParams[i].instId;
            }
        }


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
        EnetApp_createPhyRegisterPollingTask(1000, true, enetTypeList, instIdList);
#else
        EnetApp_mdioLinkIntHandlerTask(enetTypeList, instIdList);

        for (i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
        {
            EnetApp_enableMdioStateMachine(gEnetAppParams[i].enetType, gEnetAppParams[i].instId);
        }
#endif
        portENTER_CRITICAL();

        /* Wait for PHY to become alive and then initialize PHY driver */
        for (i = 0; i < ENET_SYSCFG_MAX_ENET_INSTANCES; i++)
        {
            for (uint32_t portIdx = 0; portIdx < gEnetAppParams[i].numMacPort; portIdx++)
            {
                EnetApp_waitForPhyAlive(gEnetAppParams[i].enetType,
                                         gEnetAppParams[i].instId,
                                         EnetSoc_getCoreId(),
                                         gEnetAppParams[i].macPortList[portIdx]);

                EnetApp_initExtPhy(gEnetAppParams[i].enetType,
                                    gEnetAppParams[i].instId,
                                    gEnetAppParams[i].macPortList[portIdx]);

            }

        }

        /* wait for atleast one port link to come up */
        islinked   = false;
        phyAddMask = 0;
        do
        {
            for (i = 0; i < ENETAPP_EXT_PHY_NUM_ENABLED_PORTS; i++)
            {
                const EnetExtPhy_Handle hPhy = EnetApp_getExtPhyHandle(i);
                phyAddMask |= (1 << hPhy->phyCfg.phyAddr);
                islinked   |= EnetExtPhy_WaitForLinkUp(hPhy, ENETEXTPHY_TIMEOUT_MS);
            }
        } while (!islinked);

#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
        EnetApp_enablePhyLinkPollingMask(phyAddMask);
#else
       (void)phyAddMask;
#endif
        portEXIT_CRITICAL();
    }
    #endif // #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
    /* no stdio-buffering, please! */
    //setvbuf(stdout, NULL,_IONBF, 0);
    main_loop(NULL);
    return 0;
}
#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)

EnetExtPhy_Handle ghExtPhy[ENET_SYSCFG_MAX_MAC_PORTS] = {NULL};

EnetExtPhy_Handle EnetApp_getExtPhyHandle(uint32_t portIdx)
{
    if (portIdx < ENET_SYSCFG_MAX_MAC_PORTS)
    {
        return ghExtPhy[portIdx];
    }
    else
    {
        DebugP_log("invalid mac portIdx: %d\n\r", portIdx);
        return NULL;
    }
}

static void EnetApp_initExtPhyArgs(Enet_Type enetType,
                            uint32_t instId,
                            EnetExtPhy_Mii *mii,
                            EnetExtPhy_LinkCfg *linkCfg,
                            Enet_MacPort macPort,
                            EnetExtPhy_Cfg *phyCfg)
{
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;

    /* Setup board for requested Ethernet port */
    ethPort.instId   = instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = EnetBoard_getId();
    ethPort.enetType = enetType;
    EnetBoard_getMiiConfig(&ethPort.mii);
    *mii = (EnetExtPhy_Mii) EnetUtils_macToPhyMii(&ethPort.mii);

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL)
    {
        EnetExtPhy_initCfg(phyCfg);
        phyCfg->phyAddr     = boardPhyCfg->phyAddr;
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

    linkCfg->speed     = ENETEXTPHY_SPEED_AUTO;
    linkCfg->duplexity = ENETEXTPHY_DUPLEX_AUTO;

#if (ENET_SYSCFG_ENABLE_GIGABIT == 0U)
    {
        /* If gigabit support is disabled at instance level, update the phy userCaps to
            * not advertise Gigabit capability so that link is not established at gigabit speed
            */
        if ((linkCfg->speed == ENETPHY_SPEED_AUTO) ||
            (linkCfg->duplexity == ENETPHY_DUPLEX_AUTO))
        {
            if (phyCfg->nwayCaps == 0u)
            {
                phyCfg->nwayCaps = ENETPHY_LINK_CAP_ALL;
            }
            phyCfg->nwayCaps &= ~(ENETPHY_LINK_CAP_1000);
        }
        else
        {
            /* Application has disabled Gigabit support in syscfg but
                * is configuring link speed expliticly to gigabit.
                * This is wrong configuration. Fix sysconfig to enable
                * gigabit support
                */
            EnetAppUtils_assert(linkCfg->speed != ENETPHY_SPEED_1GBIT);
        }
    }
#endif

}

static void EnetApp_initExtPhy(Enet_Type enetType,
                               uint32_t instId,
                               Enet_MacPort macPort)
{
    Enet_Handle hEnet;
    EnetExtPhy_Mii mii;
    EnetExtPhy_LinkCfg linkCfg;
    EnetExtPhy_Cfg phyCfg;
    EnetExtPhy_MdioHandle hMdio;
    uint32_t macPortCaps;

    hEnet = Enet_getHandle(enetType, instId);
    EnetAppUtils_assert(hEnet != NULL);
    hMdio = EnetExtPhyMdioDflt_getPhyMdio();
    EnetAppUtils_assert(hEnet != NULL);
    macPortCaps = EnetSoc_getMacPortCaps(enetType, instId, macPort);

    EnetApp_initExtPhyArgs(enetType,
                           instId,
                           &mii,
                           &linkCfg,
                           macPort,
                           &phyCfg);
    ghExtPhy[ENET_MACPORT_NORM(macPort)] = EnetExtPhy_open(&phyCfg,
                               mii,
                               &linkCfg,
                               macPortCaps,
                               hMdio,
                               hEnet);
    EnetAppUtils_assert(ghExtPhy[ENET_MACPORT_NORM(macPort)] != NULL);

}

int32_t   EnetApp_getExtPhyLinkCfgInfo(Enet_Type enetType,
                                       uint32_t instId,
                                       Enet_MacPort *macPort,
                                       uint32_t phyAddr,
                                       EnetPhy_LinkCfg *phyLinkCfg)
{
    EnetExtPhy_LinkCfg extPhylinkCfg;
    int32_t status;
    uint8_t numMacPorts;
    uint32_t i;
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];

    EnetApp_getEnetInstMacInfo(enetType,
                               instId,
                               macPortList,
                               &numMacPorts);

    for (i = 0; i < numMacPorts ; i++)
    {
        EnetBoard_EthPort ethPort;
        const EnetBoard_PhyCfg *boardPhyCfg;

        /* Setup board for requested Ethernet port */
        ethPort.instId   = instId;
        ethPort.macPort  = macPortList[i];
        ethPort.boardId  = ENETBOARD_AM64X_AM243X_EVM;
        ethPort.enetType = enetType;
        ethPort.mii.layerType      = ENET_MAC_LAYER_GMII;
        ethPort.mii.sublayerType   = ENET_MAC_SUBLAYER_REDUCED;
        ethPort.mii.variantType    = ENET_MAC_VARIANT_FORCED;
        boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
        EnetAppUtils_assert(boardPhyCfg != NULL);
        if (phyAddr == boardPhyCfg->phyAddr)
        {
            break;
        }
    }
    EnetAppUtils_assert (i < numMacPorts);
    *macPort = macPortList[i];
    status = EnetExtPhy_getLinkCfg(ghExtPhy[ENET_MACPORT_NORM(*macPort)], &extPhylinkCfg);
    if (status == ENETEXTPHY_SOK)
    {
        phyLinkCfg->speed     =  (EnetPhy_Speed) extPhylinkCfg.speed;
        phyLinkCfg->duplexity =  (EnetPhy_Duplexity) extPhylinkCfg.duplexity;
    }
    return status;
}

bool EnetApp_isPortLinked(Enet_Handle hEnet)
{
    Enet_Type enetType;
    uint32_t instId;
    uint32_t linkUpMask = 0U;
    bool linkUp;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;
    int32_t status;
    uint32_t i;

    status = Enet_getHandleInfo(hEnet, &enetType, &instId);
    EnetAppUtils_assert(status == ENET_SOK);
    EnetApp_getEnetInstMacInfo(enetType, instId, macPortList, &numMacPorts);
    for (i = 0; i < ENETAPP_EXT_PHY_NUM_ENABLED_PORTS; i++)
    {
        linkUpMask |= (EnetExtPhy_isLinked(ghExtPhy[i])  << i);
    }
    linkUp =  (linkUpMask != 0) ? true : false;
    return linkUp;
}

static void EnetApp_enableMdioStateMachine(Enet_Type enetType,
                                           uint32_t instId,
                                           uint32_t coreId)
{
    Enet_IoctlPrms prms;
    int32_t status;
    Enet_Handle hEnet = Enet_getHandle(enetType, instId);

    EnetAppUtils_assert(hEnet != NULL);
    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(hEnet, coreId, ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE, &prms, status);
    if (status != ENET_SOK)
    {
       EnetAppUtils_print("EnetApp_enableMdioStateMachine() failed ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE: %d\n",
                           status);
    }
    EnetAppUtils_assert(ENET_SOK == status);
}

static void EnetApp_waitForPhyAlive(Enet_Type enetType,
                                    uint32_t instId,
                                    uint32_t coreId,
                                    Enet_MacPort macPort)
{
    bool alive;
    Enet_IoctlPrms prms;
    int32_t status;
    Enet_Handle hEnet = Enet_getHandle(enetType, instId);
    uint32_t phyAddr;
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;

    EnetAppUtils_assert(hEnet != NULL);
    /* Setup board for requested Ethernet port */
    ethPort.instId   = instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = ENETBOARD_AM64X_AM243X_EVM;
    ethPort.enetType = enetType;
    ethPort.mii.layerType      = ENET_MAC_LAYER_GMII;
    ethPort.mii.sublayerType   = ENET_MAC_SUBLAYER_REDUCED;
    ethPort.mii.variantType    = ENET_MAC_VARIANT_FORCED;
    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    EnetAppUtils_assert(boardPhyCfg != NULL);
    phyAddr = boardPhyCfg->phyAddr;

    do {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, &alive);
        ENET_IOCTL(hEnet,
                   coreId,
                   ENET_MDIO_IOCTL_IS_ALIVE,
                   &prms,
                   status);
    } while ((status == ENET_SOK) && (alive != true));
    EnetAppUtils_assert (status == ENET_SOK);
}
#endif /* #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U) */

