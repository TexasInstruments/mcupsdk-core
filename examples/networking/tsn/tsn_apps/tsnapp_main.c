/*
 *  Copyright (c) Texas Instruments Incorporated 2022
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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
 * \file  tsnapp_main.c
 *
 * \brief This file contains the implementation of the Enet TSN example.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <enet.h>
#include <kernel/dpl/ClockP.h>
#include "FreeRTOS.h"
#include <kernel/dpl/TaskP.h>
#include <task.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <tsn_combase/combase.h>
#include "tsn_gptp/gptpconf/gptpgcfg.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
/* include this header is very important, otherwise some macro is not exported here.
 * e.g. ENET_SYSCFG_ENABLE_MDIO_MANUALMODE */
#include "ti_enet_config.h"
#include "debug_log.h"
#include "tsninit.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MAX_NUM_MAC_PORTS (3U)

/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

/* Test parameters for each port in the multi-channel test */
typedef struct EnetApp_Cfg_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    char *name;

    /* Number of Ethernet ports to be enabled */
    uint8_t numMacPorts;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPorts[MAX_NUM_MAC_PORTS];
} EnetApp_Cfg;

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */

static EnetApp_Cfg gEnetAppCfg =
{
    .name = "sitara-cpsw",
};

#define EnetAppAbort(message) \
    EnetAppUtils_print(message);                \
    EnetAppUtils_assert(false);

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
static char g_netdevices[MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

static void update_enet_appcfg(EnetApp_Cfg *enet_cfg)
{
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enet_cfg->enetType, &enet_cfg->instId);
    EnetApp_getEnetInstMacInfo(enet_cfg->enetType, enet_cfg->instId,
                               enet_cfg->macPorts, &enet_cfg->numMacPorts);
}

#define LOG_BUFFER_SIZE (1024)
void console_print(const char *pcString, ...)
{
    /* Use DebugP_log() because EnetAppUtils_print() has limit bufsize */
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

static int lldcfg_update_callback(cb_socket_lldcfg_update_t *update_cfg)
{
    int res = 0;
    if (update_cfg->proto == ETH_P_1588)
    {
        update_cfg->dmaTxChId = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId = ENET_DMA_RX_CH_PTP;
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    else if (update_cfg->proto == ETH_P_NETLINK)
    {
        update_cfg->dmaTxChId = 0; // DMA is not used
        update_cfg->dmaRxChId = 0; // DMA is not used
        update_cfg->nTxPkts = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize = ENET_MEM_LARGE_POOL_PKT_SIZE;
        update_cfg->unusedDma = true;
    }
    else
    {
        EnetAppUtils_print("%s:unsupported other than PTP"LINE_FEED, __func__);
        res = -1;
    }
    return res;
}

static int app_start(void)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    int i;
    int res = 0;

    tsn_app_cfg_t app_cfg =
    {
        .console_out = console_print,
    };
    for (i = 0; i < gEnetAppCfg.numMacPorts; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        app_cfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = gEnetAppCfg.macPorts[i];
    }
    app_cfg.netdevs[i] = NULL;
    if (tsn_app_init(&app_cfg) < 0)
    {
        EnetAppAbort("Failed to tsn_app_init!"LINE_FEED);
        res = -1;
    }
    else
    {
        if (cb_lld_init_devs_table(ethdevs, i, gEnetAppCfg.enetType,
                                   gEnetAppCfg.instId) < 0)
        {
            EnetAppAbort("Failed to int devs table!"LINE_FEED);
            res = -1;
        }
        else
        {
            cb_socket_set_lldcfg_update_cb(lldcfg_update_callback);

            if (tsn_app_start() < 0)
            {
                EnetAppAbort("Failed to start gptp!"LINE_FEED);
                res =  -1;
            }
            else
            {
                EnetAppUtils_print("%s:gptp start done!"LINE_FEED, __func__);
                res = 0;
            }
        }
    }
    return res;
}

static void App_printCpuLoad()
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
    Drivers_open();
    Board_driversOpen();
    DebugP_log("=========================="LINE_FEED);
    DebugP_log("	 Excelfore TSN App	  "LINE_FEED);
    DebugP_log("=========================="LINE_FEED);

    /* To support gptp switch mode, we must configure from syscfg file:
     * enet_cpsw1.DisableMacPort2 = false; */
    update_enet_appcfg(&gEnetAppCfg);

    EnetAppUtils_enableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    DebugP_log("start to open driver."LINE_FEED);
    EnetApp_driverInit();
    EnetApp_driverOpen(gEnetAppCfg.enetType, gEnetAppCfg.instId);

    if (app_start())
    {
        DebugP_log("app_start failed"LINE_FEED);
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
        tsn_app_stop();
        tsn_app_deinit();
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
        EnetAppUtils_print("Invalid MAC mode: %u"LINE_FEED, macMode);
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

    EnetAppUtils_print("%s: Open port %u"LINE_FEED,
                       gEnetAppCfg.name, ENET_MACPORT_ID(macPort));

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gEnetAppCfg.enetType;
    ethPort.instId = gEnetAppCfg.instId;
    ethPort.macPort = macPort;
    ethPort.boardId = EnetBoard_getId();
    EnetApp_macMode2MacMii(RGMII, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    if (status != ENET_SOK) {
        EnetAppUtils_print("%s: Failed to setup MAC port %u"LINE_FEED,
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
        EnetAppUtils_print("%s: No PHY configuration found"LINE_FEED, gEnetAppCfg.name);
        EnetAppUtils_assert(false);
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp, void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s"LINE_FEED,
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

    cpswCfg->portLinkStatusChangeCb	= &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs"LINE_FEED);
    EnetAppUtils_print("----------------------------------------------"LINE_FEED);
    EnetAppUtils_print("%s: init config"LINE_FEED, gEnetAppCfg.name);

    cpswCfg->vlanCfg.vlanAware = false;
    cpswCfg->hostPortCfg.removeCrc = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors = true;
    EnetApp_initEnetLinkCbPrms(cpswCfg);
    EnetApp_initAleConfig(&cpswCfg->aleCfg);
}
