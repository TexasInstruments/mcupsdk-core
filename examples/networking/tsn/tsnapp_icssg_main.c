/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  tsnapp_icssg_main.c
 *
 * \brief This file contains the implementation of the Enet ICSSG TSN example.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <tsn_combase/combase.h>
#include "nrt_flow/dataflow.h"
#include "debug_log.h"
#include "tsninit.h"
#include "enetapp_icssg.h"

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */

EnetApp_Cfg gEnetAppCfg;

#define EnetAppAbort(message) \
    EnetAppUtils_print(message);                \
    EnetAppUtils_assert(false);

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
static char g_netdevices[MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

static void EnetApp_updateCfg(void)
{
    uint32_t i = 0U;

    for (i = 0U; i < gEnetAppCfg.numPerCtxts; i++)
    {
        gEnetAppCfg.perCtxt[i].name = "sitara-icssg";
        EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0 + i,
                                &(gEnetAppCfg.perCtxt[i].enetType),
                                &(gEnetAppCfg.perCtxt[i].instId));

        EnetApp_getEnetInstMacInfo(gEnetAppCfg.perCtxt[i].enetType,
                                   gEnetAppCfg.perCtxt[i].instId,
                                   gEnetAppCfg.perCtxt[i].macPort,
                                   &(gEnetAppCfg.perCtxt[i].macPortNum));

        EnetApp_getNonPtpRxDmaInfo(gEnetAppCfg.perCtxt[i].enetType,
                                   gEnetAppCfg.perCtxt[i].instId,
                                   gEnetAppCfg.perCtxt[i].nonPtpRxFlowId,
                                   &gEnetAppCfg.perCtxt[i].nonPtpRxFlowNum);

        EnetApp_getNonPtpTxDmaInfo(gEnetAppCfg.perCtxt[i].enetType,
                                   gEnetAppCfg.perCtxt[i].instId,
                                   gEnetAppCfg.perCtxt[i].nonPtpTxChId,
                                   &gEnetAppCfg.perCtxt[i].nonPtpTxChNum);
    }
    gEnetAppCfg.gptpPerIdx = 0U;
#if ENET_SYSCFG_DUAL_MAC
    /* Fill the PTP MAC Port to the app cfg. In Dual Mac mode, only Mac Port 1 is supported currently. */
    gEnetAppCfg.numPtpPorts = 1U;
    gEnetAppCfg.ptpMacPorts[0] = ENET_MAC_PORT_1;
#else
    /* ICSSG switch case */
    /* Fill the PTP MAC Ports to the app cfg */
    EnetAppUtils_assert(gEnetAppCfg.numPerCtxts == 1U);
    gEnetAppCfg.numPtpPorts = gEnetAppCfg.perCtxt[0].macPortNum;
    for (i = 0U; i < gEnetAppCfg.numPtpPorts; i++)
    {
        gEnetAppCfg.ptpMacPorts[i] = gEnetAppCfg.perCtxt[0].macPort[i];
    }
#endif
}

#define LOG_BUFFER_SIZE (1024)
void ConsolePrint(const char *pcString, ...)
{
    /* Use DebugP_log() because EnetAppUtils_print() has limit bufsize */
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

static int EnetApp_initTsn(void)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    int i;
    int res = 0;
    AppTsnCfg_t appCfg =
    {
        .consoleOutCb = ConsolePrint,
    };

    for (i = 0; i < gEnetAppCfg.numPtpPorts; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = gEnetAppCfg.ptpMacPorts[i];
        if (i == 0)
        {
            /* tilld0 reuses the allocated source mac, other interfaces will allocate
             * the mac by themself */
            memcpy(ethdevs[i].srcmac, gEnetAppCfg.perCtxt[gEnetAppCfg.gptpPerIdx].macAddr, ENET_MAC_ADDR_LEN);
        }
    }
    appCfg.netdevs[i] = NULL;
    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppAbort("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, gEnetAppCfg.perCtxt[gEnetAppCfg.gptpPerIdx].enetType,
                               gEnetAppCfg.perCtxt[gEnetAppCfg.gptpPerIdx].instId) < 0)
    {
        EnetAppAbort("Failed to int devs table!\r\n");
    }
    cb_socket_set_lldcfg_update_cb(EnetApp_lldCfgUpdateCb);

    if (EnetApp_startTsn() < 0)
    {
        EnetAppAbort("Failed to start TSN App!\r\n");
    }
    EnetAppUtils_print("%s:TSN app start done!\r\n", __func__);

    return res;
}

static void App_printCpuLoad(void)
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ( (currTime_ms - startTime_ms) > printInterval_ms )
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                    currTime_ms/1000, currTime_ms%1000,
                    cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

void EnetApp_mainTask(void *args)
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;
    int32_t status = ENET_SOK;
    uint32_t i = 0U;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("          gPTP App        \r\n");
    DebugP_log("==========================\r\n");

    memset(&gEnetAppCfg, 0, sizeof(gEnetAppCfg));
    gEnetAppCfg.numPerCtxts = ENET_SYSCFG_NUM_PERIPHERAL;

    EnetApp_updateCfg();

    gEnetAppCfg.coreId = EnetSoc_getCoreId();
    EnetQueue_initQ(&gEnetAppCfg.txFreePktInfoQ);
    for (i = 0U; i < gEnetAppCfg.numPerCtxts; i++)
    {
        gEnetAppCfg.perCtxt[i].perIdx= i;
        EnetAppUtils_enableClocks(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId);
    }
    DebugP_log("start to open driver.\r\n");
    EnetApp_driverInit();
    for (i = 0U; i < gEnetAppCfg.numPerCtxts; i++)
    {
        status = EnetApp_driverOpen(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
        EnetApp_acquireHandleInfo(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId, &handleInfo);
        gEnetAppCfg.perCtxt[i].hEnet = handleInfo.hEnet;
        EnetApp_coreAttach(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId, gEnetAppCfg.coreId, &attachCoreOutArgs);
        gEnetAppCfg.perCtxt[i].coreKey = attachCoreOutArgs.coreKey;
        EnetAppUtils_print("%s perCtxt %d: Create RX task for regular traffic \r\n", gEnetAppCfg.perCtxt[i].name, i);
        EnetApp_createRxTask(&gEnetAppCfg.perCtxt[i]);
    }

    if (EnetApp_initTsn())
    {
        DebugP_log("EnetApp_initTsn failed\r\n");
    }
    else
    {
        while (true)
        {
            // Print CPU load
            ClockP_usleep(1000);
            App_printCpuLoad();
            TaskP_yield();
        }
        EnetApp_stopTsn();
        EnetApp_deInitTsn();
    }
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
    #if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = NULL;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg = NULL;
    #else
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = &EnetApp_mdioLinkStatusChange;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg = NULL;
    EnetApp_updateMdioLinkIntCfg(enetType, instId, &icssgCfg->mdioLinkIntCfg);
    #endif
}

static void EnetApp_closePort(EnetApp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < numPerCtxts; i++)
    {
        for (uint32_t idx = 0U; idx < perCtxts[i].macPortNum; idx++)
        {
            macPort = perCtxts[i].macPort[idx];
            EnetAppUtils_print("%s: Close port %u\r\n", perCtxts[i].name, ENET_MACPORT_ID(macPort));
            /* Close port link */
            ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
            EnetAppUtils_print("%s: Close port %u link\r\n", perCtxts[i].name, ENET_MACPORT_ID(macPort));
            ENET_IOCTL(perCtxts[i].hEnet, gEnetAppCfg.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to close port link: %d\r\n", perCtxts[i].name, status);
            }
        }
    }
}

void EnetApp_close()
{
    uint32_t numPerCtxts = gEnetAppCfg.numPerCtxts;
    uint32_t i = 0U;

    EnetAppUtils_print("\nClose Ports for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_closePort(gEnetAppCfg.perCtxt, numPerCtxts);

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for(i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_destroyRxTask(&gEnetAppCfg.perCtxt[i]);
    }

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for(i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_coreDetach(gEnetAppCfg.perCtxt[i].enetType,
                           gEnetAppCfg.perCtxt[i].instId,
                           gEnetAppCfg.coreId,
                           gEnetAppCfg.perCtxt[i].coreKey);
    }

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for(i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_releaseHandleInfo(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId);
        gEnetAppCfg.perCtxt[i].hEnet = NULL;
    }

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for(i = 0U; i < numPerCtxts; i++)
    {
        EnetAppUtils_disableClocks(gEnetAppCfg.perCtxt[i].enetType, gEnetAppCfg.perCtxt[i].instId);
    }
}
