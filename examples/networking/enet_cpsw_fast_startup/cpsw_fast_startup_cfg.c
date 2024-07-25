/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 * \file  cpsw_fast_startup_cfg.c
 *
 * \brief This file contains the implementation of the APIs for peripheral configuration for Enet Fast Startup example
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "cpsw_fast_startup_common.h"
#include "cpsw_fast_startup_cfg.h"
#include "cpsw_fast_startup_dataflow.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "enet_profiler.h"
#include <enet_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_init(void)
{
    int32_t status = ENET_SOK;

    gEnetApp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);

    return status;
}

void EnetApp_deinit(void)
{
    EnetAppUtils_print("Deinit complete\r\n");

}

void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                          bool isLinkUp,
                                          void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                             void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb     = NULL;
    cpswCfg->mdioLinkStateChangeCbArg  = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb     = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg  = &gEnetApp;
#endif

    cpswCfg->portLinkStatusChangeCb    = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &gEnetApp;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId,   Cpsw_Cfg *cpswCfg)
{
    /* Prepare init configuration for all peripherals */
    cpswCfg->vlanCfg.vlanAware          = false;
    cpswCfg->hostPortCfg.removeCrc      = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors  = true;
    EnetApp_initEnetLinkCbPrms(cpswCfg);

}

int32_t EnetApp_open(void)
{
    int32_t status = ENET_SOK;
    EnetApp_HandleInfo handleInfo;
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;

    /* Do peripheral dependent initalization */
    EnetAppUtils_enableClocks(gEnetApp.enetType, gEnetApp.instId);

    EnetApp_driverInit();

    /* Open Enet driver for all peripherals */
    status = EnetApp_driverOpen(gEnetApp.enetType, gEnetApp.instId);

    EnetApp_acquireHandleInfo(gEnetApp.enetType, gEnetApp.instId, &handleInfo);
    gEnetApp.hEnet = handleInfo.hEnet;

    EnetApp_coreAttach(gEnetApp.enetType, gEnetApp.instId, gEnetApp.coreId, &attachCoreOutArgs);
    gEnetApp.coreKey = attachCoreOutArgs.coreKey;

    /* Open DMA for peripheral/port */
    status = EnetApp_openDma();

    return status;
}

void EnetApp_close(void)
{
    /* Close Ports for all peripherals */
    EnetApp_closePort();

    /* Close DMA for peripheral/port */
    EnetApp_closeDma();

    /* Detach core */
    EnetApp_coreDetach(gEnetApp.enetType, 
                       gEnetApp.instId,
                       gEnetApp.coreId,
                       gEnetApp.coreKey);

    /* Close opened Enet drivers if any peripheral failed */
    EnetApp_releaseHandleInfo(gEnetApp.enetType, gEnetApp.instId);
    gEnetApp.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral Clocks */
    EnetAppUtils_disableClocks(gEnetApp.enetType, gEnetApp.instId);

}

void EnetApp_closePort(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        macPort = gEnetApp.macPort[i];

        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
        
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to close port link: %d\r\n", gEnetApp.name, status);
        }
    }
}

int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: Waiting for link up...\r\n", gEnetApp.name);

    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        macPort = gEnetApp.macPort[i];
        linked = false;

        while (gEnetApp.run && !linked)
        {
            EnetApp_phyStateHandler();
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u link status: %d\r\n",
                                    gEnetApp.name, ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                ClockP_usleep(3000U);
            }
            else
            {
                PROFILE_TIME(ClockP_getTimeUsec());
            }
        }
    }
    return status;
}
