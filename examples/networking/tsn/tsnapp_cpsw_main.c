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
 * \file  tsnapp_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet CPSW TSN example.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <tsn_combase/combase.h>
#include "nrt_flow/dataflow.h"
#include "debug_log.h"
#include "tsninit.h"
#include "enetapp_cpsw.h"

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */

EnetApp_Cfg gEnetAppCfg =
{
    .name = "sitara-cpsw",
};

#define EnetAppAbort(message) \
    EnetAppUtils_print(message);                \
    EnetAppUtils_assert(false);

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
static char g_netdevices[MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

static void EnetApp_updateCfg(EnetApp_Cfg *enet_cfg)
{
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enet_cfg->enetType, &enet_cfg->instId);
    EnetApp_getEnetInstMacInfo(enet_cfg->enetType, enet_cfg->instId,
                               enet_cfg->macPorts, &enet_cfg->numMacPorts);
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

    for (i = 0; i < gEnetAppCfg.numMacPorts; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = gEnetAppCfg.macPorts[i];
        if (i == 0)
        {
            /* tilld0 reuses the allocated source mac, other interfaces will allocate
             * the mac by themself */
            memcpy(ethdevs[i].srcmac, gEnetAppCfg.macAddr, ENET_MAC_ADDR_LEN);
        }
    }
    appCfg.netdevs[i] = NULL;
    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppAbort("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, gEnetAppCfg.enetType,
                               gEnetAppCfg.instId) < 0)
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

static bool IsMacAddrSet(uint8_t *mac)
{
    return ((mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5]) != 0);
}

uint32_t EnetApp_applyClassifier(Enet_Handle hEnet, uint32_t coreId, uint8_t *dstMacAddr, uint32_t vlanId,
                                    uint32_t ethType, uint32_t rxFlowIdx)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    if (IsMacAddrSet(dstMacAddr) == true)
    {
        status = EnetAppUtils_addHostPortMcastMembership(hEnet, dstMacAddr);
        if (status != ENET_SOK) {
            EnetAppUtils_print("%s:EnetAppUtils_addHostPortMcastMembership failed: %d\r\n", gEnetAppCfg.name, status);
        }
    }
    memset(&setPolicerEntryInArgs, 0, sizeof (setPolicerEntryInArgs));

    if (ethType > 0) {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
            CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = ethType;
    }
    setPolicerEntryInArgs.policerMatch.portIsTrunk = false;
    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = rxFlowIdx;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs, &setPolicerEntryOutArgs);
    ENET_IOCTL(hEnet, coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);
    return status;
}

static void EnetApp_enableTsSync()
{
    Enet_IoctlPrms prms;
    CpswCpts_OutputBitSel bitSelect;
    int32_t status;

    bitSelect = CPSW_CPTS_TS_OUTPUT_BIT_17;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to set TS SYNC OUT BIT : %d\r\n", gEnetAppCfg.name, status);
    }
    return;
}

void EnetApp_mainTask(void *args)
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("          gPTP App        \r\n");
    DebugP_log("==========================\r\n");

    /* To support gptp switch mode, we must configure from syscfg file:
     * enet_cpsw1.DisableMacPort2 = false; */
    EnetApp_updateCfg(&gEnetAppCfg);

    gEnetAppCfg.coreId = EnetSoc_getCoreId();
    EnetQueue_initQ(&gEnetAppCfg.txFreePktInfoQ);
    EnetAppUtils_enableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    DebugP_log("start to open driver.\r\n");
    EnetApp_driverInit();
    EnetApp_driverOpen(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    EnetApp_acquireHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId, &handleInfo);
    gEnetAppCfg.hEnet = handleInfo.hEnet;
    EnetApp_coreAttach(gEnetAppCfg.enetType, gEnetAppCfg.instId, gEnetAppCfg.coreId, &attachCoreOutArgs);
    gEnetAppCfg.coreKey = attachCoreOutArgs.coreKey;
    EnetAppUtils_print("%s: Create RX task for regular traffic \r\n", gEnetAppCfg.name);
    EnetApp_enableTsSync();
    EnetApp_createRxTask();

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

static void EnetApp_macMode2MacMii(emac_mode macMode, EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
    case RMII:
        mii->layerType = ENET_MAC_LAYER_MII;
        mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
        mii->variantType = ENET_MAC_VARIANT_NONE;
        break;

    case RGMII:
        mii->layerType = ENET_MAC_LAYER_GMII;
        mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
        mii->variantType = ENET_MAC_VARIANT_FORCED;
        break;
    default:
        EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
        EnetAppUtils_assert(false);
        break;
    }
}

void EnetApp_initLinkArgs(Enet_Type enetType, uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs, Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: Open port %u\r\n",
                       gEnetAppCfg.name, ENET_MACPORT_ID(macPort));

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gEnetAppCfg.enetType;
    ethPort.instId = gEnetAppCfg.instId;
    ethPort.macPort = macPort;
    ethPort.boardId = EnetBoard_getId();
    EnetApp_macMode2MacMii(RGMII, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    if (status != ENET_SOK) {
        EnetAppUtils_print("%s: Failed to setup MAC port %u\r\n",
                           gEnetAppCfg.name, ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(false);
    }

    /* Set port link params */
    linkArgs->macPort = macPort;

    mii->layerType = ethPort.mii.layerType;
    mii->sublayerType = ethPort.mii.sublayerType;
    mii->variantType = ENET_MAC_VARIANT_FORCED;

    linkCfg->speed = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL) {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr = boardPhyCfg->phyAddr;
        phyCfg->isStrapped = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn = false;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        /* Setting Tx and Rx skew delay values */
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);

    } else {
        EnetAppUtils_print("%s: No PHY configuration found\r\n", gEnetAppCfg.name);
        EnetAppUtils_assert(false);
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp, void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                             void *appArg)
{
}

static void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;

    aleCfg->nwSecCfg.vid0ModeEn = true;
    aleCfg->vlanCfg.aleVlanAwareMode = false;
    aleCfg->vlanCfg.cpswVlanAwareMode = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn = true;
    aleCfg->policerGlobalCfg.yellowDropEn = false;
    /* Enables the ALE to drop the red colored packets. */
    aleCfg->policerGlobalCfg.redDropEn = true;
    /* Policing match mode */
    aleCfg->policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
}

static void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif

    cpswCfg->portLinkStatusChangeCb    = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("%s: init config\r\n", gEnetAppCfg.name);

    cpswCfg->vlanCfg.vlanAware = false;
    cpswCfg->hostPortCfg.removeCrc = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors = true;
    EnetApp_initEnetLinkCbPrms(cpswCfg);
    EnetApp_initAleConfig(&cpswCfg->aleCfg);
#ifdef SOC_AM263X
    EnetCpdma_Cfg *dmaCfg;
    /* Set the enChOverrideFlag to enable the channel override feature of CPDMA */
    dmaCfg=(EnetCpdma_Cfg *)cpswCfg->dmaCfg;
    dmaCfg->enChOverrideFlag = true;
#endif
}

static void EnetApp_closePort()
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < gEnetAppCfg.numMacPorts; i++)
    {
        macPort = gEnetAppCfg.macPorts[i];

        EnetAppUtils_print("%s: Close port %u\r\n", gEnetAppCfg.name, ENET_MACPORT_ID(macPort));

        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);

        EnetAppUtils_print("%s: Close port %u link\r\n", gEnetAppCfg.name, ENET_MACPORT_ID(macPort));
        ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to close port link: %d\r\n", gEnetAppCfg.name, status);
        }
    }
}

void EnetApp_close()
{
    EnetAppUtils_print("\nClose Ports for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_closePort();

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_destroyRxTask();

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_coreDetach(gEnetAppCfg.enetType,gEnetAppCfg.instId,
                       gEnetAppCfg.coreId,
                       gEnetAppCfg.coreKey);

    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_releaseHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    gEnetAppCfg.hEnet = NULL;

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_disableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
}
