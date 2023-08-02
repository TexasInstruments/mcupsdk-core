/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <tsn_combase/combase.h>
#include "tsn_gptp/gptpconf/gptpgcfg.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "debug_log.h"
#include "tsninit.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MAX_NUM_MAC_PORTS (3U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

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

    /* This core's id */
    uint32_t coreId;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Start flow index */
    uint32_t rxStartFlowIdx;

    /* Regular traffic RX flow index */
    uint32_t rxFlowIdx;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* Number of Ethernet ports to be enabled */
    uint8_t numMacPorts;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all Regular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPorts[MAX_NUM_MAC_PORTS];
} EnetApp_Cfg;

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */

static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

/* Rx Task for Regular non-gPTP traffic */
void EnetApp_createRxTask();

void EnetApp_rxTask(void *args);

/* Get DMA handle for non-gPTP channels */
int32_t EnetApp_openDma();

void EnetApp_initTxFreePktQ();

uint32_t EnetApp_retrieveFreeTxPkts();

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

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
        memcpy(ethdevs[i].srcmac, gEnetAppCfg.macAddr, ENET_MAC_ADDR_LEN);
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
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;
    int32_t status = ENET_SOK;

    DebugP_log("=========================="LINE_FEED);
    DebugP_log("         TSN App          "LINE_FEED);
    DebugP_log("=========================="LINE_FEED);

    /* To support gptp switch mode, we must configure from syscfg file:
     * enet_cpsw1.DisableMacPort2 = false; */
    update_enet_appcfg(&gEnetAppCfg);

    gEnetAppCfg.coreId = EnetSoc_getCoreId();
    EnetQueue_initQ(&gEnetAppCfg.txFreePktInfoQ);
    EnetAppUtils_enableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    EnetAppUtils_print("%s: Create RX task for regular traffic \r\n", gEnetAppCfg.name);
    EnetApp_createRxTask();
    DebugP_log("start to open driver."LINE_FEED);
    EnetApp_driverInit();
    EnetApp_driverOpen(gEnetAppCfg.enetType, gEnetAppCfg.instId);
    EnetApp_acquireHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId, &handleInfo);
    gEnetAppCfg.hEnet = handleInfo.hEnet;
    EnetApp_coreAttach(gEnetAppCfg.enetType, gEnetAppCfg.instId, gEnetAppCfg.coreId, &attachCoreOutArgs);
    gEnetAppCfg.coreKey = attachCoreOutArgs.coreKey;
    /* Get DMA Channel handles for non-gPTP traffic */
    status = EnetApp_openDma();
    if (status != ENET_SOK)
    {
       EnetAppUtils_print("%s: failed to open non-gPTP DMA channels: %d\r\n", gEnetAppCfg.name, status);
    }
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
    static uint32_t linkUpCount = 0;
    if ((info->linkChanged) && (info->isLinked))
    {
        linkUpCount++;
    }
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

/* Rx Isr for non-gPTP traffic */
void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetAppCfg.rxSemObj);
}

int32_t EnetApp_openDma()
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0),
                           &txInArgs,
                           &txChInfo);

    gEnetAppCfg.txChNum = txChInfo.txChNum;
    gEnetAppCfg.hTxCh   = txChInfo.hTxCh;

    if (gEnetAppCfg.hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetAppCfg.hEnet,
                              gEnetAppCfg.coreKey,
                              gEnetAppCfg.coreId,
                              gEnetAppCfg.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetAppCfg.hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = NULL;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                               &rxInArgs,
                               &rxChInfo);

        gEnetAppCfg.rxStartFlowIdx = rxChInfo.rxFlowStartIdx;
        gEnetAppCfg.rxFlowIdx = rxChInfo.rxFlowIdx;
        gEnetAppCfg.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        EnetAppUtils_print("MAC port addr: ");
        EnetAppUtils_printMacAddr(gEnetAppCfg.macAddr);

        if (gEnetAppCfg.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetAppCfg.hRxCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {

        EnetApp_initRxReadyPktQ(gEnetAppCfg.hRxCh);
    }

     return status;
}

void EnetApp_closeDma()
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts();

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetAppCfg.txFreePktInfoQ);
}

void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_DMA_TX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ));
}

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < ENET_DMA_RX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

uint32_t EnetApp_retrieveFreeTxPkts()
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetAppCfg.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status;

    status = SemaphoreP_constructBinary(&gEnetAppCfg.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gEnetAppTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&gEnetAppCfg.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

}

void EnetApp_destroyRxTask()
{
    SemaphoreP_destruct(&gEnetAppCfg.rxSemObj);
    TaskP_destruct(&gEnetAppCfg.rxTaskObj);
}

void EnetApp_closePort()
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < gEnetAppCfg.numMacPorts; i++)
    {
        macPort =gEnetAppCfg.macPorts[i];

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

    EnetAppUtils_print("\nClose DMA for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetApp_closeDma();

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

/* Rx Echo task for non-gPTP traffic */
void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    EthFrame *txFrame;
    uint32_t totalRxCnt = 0U;
    int32_t status = ENET_SOK;

    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetAppCfg.rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.*/
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);
        EnetQueue_initQ(&txSubmitQ);

        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(gEnetAppCfg.hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }
#if DEBUG
        EnetAppUtils_print("%s: Received %u packets\r\n", gEnetAppCfg.name, EnetQueue_getQCount(&rxReadyQ));
#endif
        totalRxCnt += EnetQueue_getQCount(&rxReadyQ);

        /* Consume the received packets and send them back */
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (rxPktInfo != NULL)
        {
            rxFrame = (EthFrame *)rxPktInfo->sgList.list[0].bufPtr;
            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_READYQ);

            /* Retrieve TX packets from driver and recycle them */
            EnetApp_retrieveFreeTxPkts();

            /* Dequeue one free TX Eth packet */
            txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);
            if (txPktInfo != NULL)
            {
                /* Fill the TX Eth frame with test content */
                txFrame = (EthFrame *)txPktInfo->sgList.list[0].bufPtr;
                memcpy(txFrame->hdr.dstMac, rxFrame->hdr.srcMac, ENET_MAC_ADDR_LEN);
                memcpy(txFrame->hdr.srcMac, &gEnetAppCfg.macAddr[0U], ENET_MAC_ADDR_LEN);
                txFrame->hdr.etherType = rxFrame->hdr.etherType;

                txPktInfo->sgList.list[0].segmentFilledLen = rxPktInfo->sgList.list[0].segmentFilledLen;
                EnetAppUtils_assert(txPktInfo->sgList.list[0].segmentAllocLen >= txPktInfo->sgList.list[0].segmentFilledLen);
                memcpy(&txFrame->payload[0U],
                        &rxFrame->payload[0U],
                        rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader));

                txPktInfo->sgList.numScatterSegments = 1;
                txPktInfo->chkSumInfo = 0U;
                txPktInfo->appPriv = &gEnetAppCfg;
                txPktInfo->tsInfo.enableHostTxTs = false;

                EnetDma_checkPktState(&txPktInfo->pktState,
                                        ENET_PKTSTATE_MODULE_APP,
                                        ENET_PKTSTATE_APP_WITH_FREEQ,
                                        ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Enqueue the packet for later transmission */
                EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
            }
            else
            {
                EnetAppUtils_print("%s: Drop due to TX pkt not available\r\n", gEnetAppCfg.name);
            }

            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_READYQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Release the received packet */
            EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }

        /* Transmit all enqueued packets */
        status = EnetDma_submitTxPktQ(gEnetAppCfg.hTxCh, &txSubmitQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to submit TX pkt queue: %d\r\n", gEnetAppCfg.name, status);
        }

        EnetAppUtils_validatePacketState(&rxFreeQ,
                                            ENET_PKTSTATE_APP_WITH_FREEQ,
                                            ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        EnetDma_submitRxPktQ(gEnetAppCfg.hRxCh, &rxFreeQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to submit RX pkt queue: %d\r\n", gEnetAppCfg.name, status);
        }
    }

    EnetAppUtils_print("%s: Received %u packets\r\n", gEnetAppCfg.name, totalRxCnt);

    EnetApp_close();
    TaskP_exit();
}
