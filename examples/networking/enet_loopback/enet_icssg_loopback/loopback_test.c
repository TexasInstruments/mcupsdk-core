/*
 *  Copyright (c) Texas Instruments Incorporated 2022-2023
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
 * \file  loopback_test.c
 *
 * \brief This file contains the loopback test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/EventP.h>
#include "loopback_cfg.h"
#include "loopback_common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports supported per context */
#define ENETMP_PORT_MAX                          (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static uint32_t EnetApp_retrieveFreeTxPkts(void);

static void EnetApp_receivePkts(EnetDma_RxChHandle hRx, EnetDma_PktQ* pRecvdPktQs);

static int32_t EnetApp_waitForLinkUp(void);

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode, EnetPhy_Mii *mii);

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                   EnetMacPort_Interface *mii);

static void EnetApp_initTxFreePktQ(void);

static void EnetApp_initRxReadyPktQ(void);

static int32_t EnetApp_openDma(void);

static void EnetApp_closeDma(void);

static uint32_t EnetApp_receiveEvents(EventP_Object *pEvent);

static void EnetApp_handleEvent(const uint32_t eventMask);

static uint32_t EnetApp_preparePktQ(EnetDma_PktQ *pPktQ);

static uint32_t EnetApp_sendPktQ(EnetDma_PktQ *pPktQ);

static void EnetApp_postTxEvent(void *pArg);

static void EnetApp_postRxEvent(void *pArg);

static int32_t  EnetApp_setMacAddress(Enet_Handle hEnet, uint8_t macAddr[ENET_MAC_ADDR_LEN], uint32_t macPort);

void EnetApp_printStats();
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
extern EnetLpbk_Obj gEnetLpbk;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_loopbackTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    if (gEnetLpbk.enetType == ENET_ICSSG_SWITCH)
    {
        EnetAppUtils_print("ICSSG_SWITCH Test\r\n");
    }
    if (gEnetLpbk.enetType == ENET_ICSSG_DUALMAC)
    {
        EnetAppUtils_print("ICSSG_DUALMAC Test\r\n");
    }

    EnetApp_driverInit();

    EnetAppUtils_enableClocks(gEnetLpbk.enetType, gEnetLpbk.instId);

    /* Create Global Event Object */
    status = EventP_construct(&gEnetLpbk.appEvents);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gEnetLpbk.coreId = EnetSoc_getCoreId();
    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetLpbk.enetType, gEnetLpbk.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gEnetLpbk.exitFlag = false;
    gEnetLpbk.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gEnetLpbk.enetType, gEnetLpbk.instId,
                              &handleInfo);
    gEnetLpbk.hEnet = handleInfo.hEnet;
    gEnetLpbk.hUdmaDrv = handleInfo.hUdmaDrv;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetLpbk.enetType, gEnetLpbk.instId,
                           gEnetLpbk.coreId, &attachCoreOutArgs);
        gEnetLpbk.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = EnetApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    EnetApp_printStats();
    /* Wait for link up */
    EnetApp_initPhyStateHandlerTask(&gEnetLpbk.appEvents);
    EnetApp_setMacAddress(gEnetLpbk.hEnet, gEnetLpbk.hostMacAddr, gEnetLpbk.macPortList[0]);

    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */

    EnetApp_postTxEvent((void*) &gEnetLpbk.appEvents);
    gEnetLpbk.totalTxCnt = 0;
    gEnetLpbk.totalRxCnt = 0;
    while (true)
    {
        const uint32_t recvdEventsMask = EnetApp_receiveEvents(&gEnetLpbk.appEvents);

        if (recvdEventsMask != AppEventId_NONE)
        {
            EnetApp_handleEvent(recvdEventsMask);
            if (gEnetLpbk.totalRxCnt >= ENETLPBK_TEST_PKT_NUM)
            {
                EnetAppUtils_print("completed\r\n");
                break;
            }
        }
    }

    EnetApp_retrieveFreeTxPkts();
    if (status == ENET_SOK)
    {
        EnetApp_printStats();
    }

    /* Close Enet DMA driver */
    EnetApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gEnetLpbk.enetType, gEnetLpbk.instId, gEnetLpbk.coreId,
                       gEnetLpbk.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gEnetLpbk.enetType, gEnetLpbk.instId);
    gEnetLpbk.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gEnetLpbk.enetType, gEnetLpbk.instId);

    /* Delete all Global Event */
    EventP_destruct(&gEnetLpbk.appEvents);

    EnetAppUtils_print("Test complete: %s\r\n",
                       (status == ENET_SOK) ? "PASS" : "FAIL");


    return status;
}

void EnetApp_printStats()
{
    /* Statistics */
    IcssgStats_MacPort gEnetMp_icssgStats;
    IcssgStats_Pa gEnetMp_icssgPaStats;
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    if (Enet_isIcssFamily(gEnetLpbk.enetType))
    {
        EnetAppUtils_print("\n - HOST PORT statistics\r\n");
        EnetAppUtils_print("--------------------------------\r\n");
        ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetMp_icssgPaStats);
        ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
                   ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get PA stats\r\n");
        }

        EnetAppUtils_printIcssgPaStats(&gEnetMp_icssgPaStats);
        EnetAppUtils_print("\r\n");

        for (uint32_t portIdx = 0; portIdx < gEnetLpbk.numMacPorts; portIdx++)
        {
            macPort = (ENET_ICSSG_DUALMAC == gEnetLpbk.enetType)? ENET_MAC_PORT_1 : gEnetLpbk.macPortList[portIdx];
            EnetAppUtils_print("\n Mac %d statistics\r\n", portIdx + 1);
            EnetAppUtils_print("--------------------------------\r\n");
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetMp_icssgStats);

            ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u stats\r\n", ENET_MACPORT_ID(macPort));
            }

            EnetAppUtils_printIcssgMacPortStats(&gEnetMp_icssgStats, false);
            EnetAppUtils_print("\r\n");
        }
    }
}

void EnetApp_resetstats()
{

    /* Statistics */
    //IcssgStats_MacPort gEnetMp_icssgStats;
    //IcssgStats_Pa gEnetMp_icssgPaStats;
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    EnetAppUtils_print("Reset statistics\r\n");

    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
               ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset  host port stats\r\n");
    }

    macPort = ENET_MAC_PORT_1;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
               ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n",
                           ENET_MACPORT_ID(macPort));
    }

    ClockP_usleep(50000);

    macPort = ENET_MAC_PORT_2;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
               ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n",
                           ENET_MACPORT_ID(macPort));
    }

    ClockP_usleep(50000);

    macPort = ENET_MAC_PORT_3;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
               ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n",
                           ENET_MACPORT_ID(macPort));
    }

}
/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_postRxEvent(void *pArg)
{

    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_RXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post Rx Event\r\n");
        EnetAppUtils_assert(false);
    }

}

static void EnetApp_postTxEvent(void *pArg)
{
    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_TXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post TX Event\r\n");
        EnetAppUtils_assert(false);
    }
}

void EnetApp_postPollEvent(void *pArg)
{
    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_PERIODIC_POLL);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post Poll Event\r\n");
        EnetAppUtils_assert(false);
    }
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any ICSSG packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetLpbk.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetLpbk.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print(
                "retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

static uint32_t EnetApp_sendPktQ(EnetDma_PktQ *pPktQ)
{
    uint32_t txCnt = EnetQueue_getQCount(pPktQ);
//    EnetAppUtils_print("Sending Tx : %d\r\n", txCnt);
    const int32_t status = EnetDma_submitTxPktQ(gEnetLpbk.hTxCh, pPktQ);

    /* Retrieve TX free packets */
    if (status != ENET_SOK)
    {
        EnetAppUtils_print(
                "EnetDma_submitTxPktQ() failed to submit pkts: %d\r\n", status);
    }

//    EnetAppUtils_print("SendTx : %d\r\n", txCnt - EnetQueue_getQCount(pPktQ));
    uint32_t pktsTransmitted = txCnt - EnetQueue_getQCount(pPktQ);
    EnetQueue_append(&gEnetLpbk.txFreePktInfoQ, pPktQ);
    return pktsTransmitted; // total num of packets sent
}

static uint32_t EnetApp_preparePktQ(EnetDma_PktQ *pktQueue)
{

    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = { 0xffU, 0xffU, 0xffU, 0xffU, 0xffU,
                                             0xffU };

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetLpbk.txFreePktInfoQ);

    uint32_t totalPktCnt = gEnetLpbk.totalTxCnt;
    if (NULL != pktInfo)
    {
        /* Fill the TX Eth frame with test content */
        frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;
        memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
        memcpy(frame->hdr.srcMac, &gEnetLpbk.hostMacAddr[0U],
               ENET_MAC_ADDR_LEN);
        frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
        memset(&frame->payload[0U], (uint8_t) (0xA5 + totalPktCnt),
               ENETLPBK_TEST_PKT_LEN);
        totalPktCnt++;
        pktInfo->sgList.list[0].segmentFilledLen = ENETLPBK_TEST_PKT_LEN
                + sizeof(EthFrameHeader);
        pktInfo->sgList.numScatterSegments = 1;
        pktInfo->chkSumInfo = 0U;
        pktInfo->appPriv = &gEnetLpbk;
        pktInfo->txPortNum = ENET_MAC_PORT_INV;
        EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                              ENET_PKTSTATE_APP_WITH_FREEQ,
                              ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Enqueue the packet for later transmission */
        EnetQueue_enq(pktQueue, &pktInfo->node);
    }

//    EnetAppUtils_print("PrepareTx : %d\r\n", EnetQueue_getQCount(pktQueue));

    return EnetQueue_getQCount(pktQueue);
}

static void EnetApp_receivePkts(EnetDma_RxChHandle hRx, EnetDma_PktQ* pRecvdPktQ)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    int32_t status = EnetDma_retrieveRxPktQ(hRx, &rxReadyQ);
    if (status == ENET_SOK)
    {
        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ
         **/
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                    ENET_PKTSTATE_MODULE_APP,
                    ENET_PKTSTATE_APP_WITH_DRIVER,
                    ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(pRecvdPktQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("receivePkts() failed to retrieve pkts: %d\r\n", status);
    }
    return;
}

static void EnetApp_handleRxPkt()
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    int32_t status = ENET_SOK;

    /* Get the packets received so far */
    for (uint32_t chIdx = 0; chIdx < ENET_SYSCFG_RX_FLOWS_NUM; chIdx++)
    {
        EnetApp_receivePkts(gEnetLpbk.hRxCh[chIdx], &gEnetLpbk.rxReadyQ);

        if (EnetQueue_getQCount(&gEnetLpbk.rxReadyQ) > 0U)
        {
            /* Consume the received packets and release them */
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetLpbk.rxReadyQ);
            while (NULL != pktInfo)
            {
                EnetDma_checkPktState(&pktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_READYQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ);

                frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;

                if (memcmp(frame->hdr.srcMac, gEnetLpbk.hostMacAddr, ENET_MAC_ADDR_LEN) == 0U)
                {
                    gEnetLpbk.totalRxCnt++;
                }
                /* Consume the packet by just printing its content */
                if (gEnetLpbk.printFrame)
                {
                    uint32_t packetPrintLen;

                    packetPrintLen = pktInfo->sgList.list[0].segmentFilledLen
                            - sizeof(EthFrameHeader);
                    EnetAppUtils_printFrame(frame, packetPrintLen);
                }

                /* Release the received packet */
                EnetQueue_enq(&gEnetLpbk.rxFreeQ, &pktInfo->node);
                pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetLpbk.rxReadyQ);
            }

            /*Submit now processed buffers */
            if (status == ENET_SOK)
            {
                EnetAppUtils_validatePacketState(&gEnetLpbk.rxFreeQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ,
                        ENET_PKTSTATE_APP_WITH_DRIVER);

                EnetDma_submitRxPktQ(gEnetLpbk.hRxCh[chIdx],
                        &gEnetLpbk.rxFreeQ);
            }
        }
    }

    return;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId,
                                Icssg_Cfg *icssgCfg)
{
    EnetRm_ResCfg *resCfg;
    uint32_t i;

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit  configs EnetType:%u, InstId :%u\r\n", enetType,
                       instId);
    EnetAppUtils_print("----------------------------------------------\r\n");

#if defined(ENET_TEST_MII_MODE)
    icssgCfg->mii.layerType    = ENET_MAC_LAYER_MII;
    icssgCfg->mii.sublayerType = ENET_MAC_SUBLAYER_STANDARD;
    icssgCfg->mii.variantType  = ENET_MAC_VARIANT_NONE;
#endif

    resCfg = &icssgCfg->resCfg;

    for (i = 0U; i < ENETMP_PORT_MAX; i++)
    {
        resCfg->macList.macAddress[i][ENET_MAC_ADDR_LEN - 1] += i;
    }
    resCfg->macList.numMacAddress = ENETMP_PORT_MAX;

}

static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    for (uint32_t portIdx = 0; portIdx < gEnetLpbk.numMacPorts; portIdx++)
    {
        bool linked = false;
        while (!linked)
        {
            EnetApp_phyStateHandler();
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetLpbk.macPortList[portIdx], &linked);

            ENET_IOCTL(gEnetLpbk.hEnet, gEnetLpbk.coreId,
                       ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                                   ENET_MACPORT_ID(gEnetLpbk.macPortList[portIdx]), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                /* wait for 50 ms and poll again*/
                ClockP_usleep(50000);
            }
        }

    }

    return status;
}

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode, EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
    case RMII:
        *mii = ENETPHY_MAC_MII_RMII;
        break;

    case RGMII:
        *mii = ENETPHY_MAC_MII_RGMII;
        break;
    default:
        status = ENET_EFAIL;
        EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
        EnetAppUtils_assert(false);
        break;
    }

    return status;
}

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                   EnetMacPort_Interface *mii)
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

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetLpbk.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

        pPktInfo = EnetMem_allocEthPkt(&gEnetLpbk,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);

        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                     ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetLpbk.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetLpbk.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;

    EnetQueue_initQ(&gEnetLpbk.rxFreeQ);
    EnetQueue_initQ(&gEnetLpbk.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (uint32_t chIdx = 0; chIdx < ENET_SYSCFG_RX_FLOWS_NUM; chIdx++)
    {
        /* Divide the total packets available in pool equally between to Rx DMA channels */
        for (uint32_t pktCount = 0U; pktCount < (ENET_SYSCFG_TOTAL_NUM_RX_PKT / 2); pktCount++)
        {
            uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

            pPktInfo = EnetMem_allocEthPkt(&gEnetLpbk,
                                           ENETDMA_CACHELINE_ALIGNMENT,
                                           ENET_ARRAYSIZE(scatterSegments),
                                           scatterSegments);

            EnetAppUtils_assert(pPktInfo != NULL);
            ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                         ENET_PKTSTATE_APP_WITH_FREEQ);
            EnetQueue_enq(&gEnetLpbk.rxFreeQ, &pPktInfo->node);

            /* Retrieve any CPSW packets which are ready */
            status = EnetDma_retrieveRxPktQ(gEnetLpbk.hRxCh[chIdx], &rxReadyQ);
            EnetAppUtils_assert(status == ENET_SOK);
            /* There should not be any packet with DMA during init */
            EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

            EnetAppUtils_validatePacketState(&gEnetLpbk.rxFreeQ,
                                             ENET_PKTSTATE_APP_WITH_FREEQ,
                                             ENET_PKTSTATE_APP_WITH_DRIVER);

            EnetDma_submitRxPktQ(gEnetLpbk.hRxCh[chIdx], &gEnetLpbk.rxFreeQ);
            EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetLpbk.rxFreeQ));
        }
    }
}

static int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs txInArgs;
        EnetApp_GetTxDmaHandleOutArgs txChInfo;

        txInArgs.cbArg = &gEnetLpbk.appEvents;
        txInArgs.notifyCb = EnetApp_postTxEvent;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0, &txInArgs, &txChInfo);

        gEnetLpbk.txChNum = txChInfo.txChNum;
        gEnetLpbk.hTxCh = txChInfo.hTxCh;

        EnetApp_initTxFreePktQ();

        if (NULL != gEnetLpbk.hTxCh)
        {
            status = ENET_SOK;
            if (ENET_SOK != status)
            {
                EnetAppUtils_print("EnetUdma_startTxCh() failed: %d\r\n",
                                   status);
                status = ENET_EFAIL;
            }
        }
        else
        {
            EnetAppUtils_print("EnetDma_openTxCh() failed to open: %d\r\n",
                               status);
            status = ENET_EFAIL;
        }
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs rxChInfo;
        EnetApp_GetDmaHandleInArgs rxInArgs;

        rxInArgs.notifyCb = EnetApp_postRxEvent;
        rxInArgs.cbArg = &gEnetLpbk.appEvents;

        /* Open First Rx Channel */
        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0, &rxInArgs, &rxChInfo);

        if (NULL == rxChInfo.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n", ENET_DMA_RX_CH0);
            EnetAppUtils_assert(false);
        }
        gEnetLpbk.hRxCh[ENET_DMA_RX_CH0] = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress > 0);
        EnetUtils_copyMacAddr(gEnetLpbk.hostMacAddr, rxChInfo.macAddr[0]);

#ifdef ENET_DMA_RX_CH1
        /* Open Second Rx Channel */
        rxInArgs.notifyCb = EnetApp_postRxEvent;
        rxInArgs.cbArg = &gEnetLpbk.appEvents;
        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH1, &rxInArgs, &rxChInfo);
        if (NULL == rxChInfo.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n", ENET_DMA_RX_CH1);
            EnetAppUtils_assert(false);
        }
        gEnetLpbk.hRxCh[ENET_DMA_RX_CH1] = rxChInfo.hRxCh;
#endif
        /* Submit all ready RX buffers to DMA.*/
        EnetApp_initRxReadyPktQ();
    }

    return status;
}

static void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;
    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetLpbk.rxReadyQ));

    /* Close RX channel - ENET_DMA_RX_CH0*/
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma(ENET_DMA_RX_CH0, gEnetLpbk.hEnet, gEnetLpbk.coreKey,
                       gEnetLpbk.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close RX channel - ENET_DMA_RX_CH1*/
#ifdef ENET_DMA_RX_CH1
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma(ENET_DMA_RX_CH1, gEnetLpbk.hEnet, gEnetLpbk.coreKey,
                       gEnetLpbk.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
#endif

    /* Close TX channel - ENET_DMA_TX_CH0 */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0, gEnetLpbk.hEnet, gEnetLpbk.coreKey,
                       gEnetLpbk.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetLpbk.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gEnetLpbk.txFreePktInfoQ);

}

static uint32_t EnetApp_receiveEvents(EventP_Object *pEvent)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t recvdEventsMask = AppEventId_NONE;

    status = EventP_waitBits(pEvent, AppEventId_ANY_EVENT, // bitsToWaitFor
                             1, 0,
                             SystemP_NO_WAIT,
                             &recvdEventsMask);

    if ((status != SystemP_SUCCESS) && (status != SystemP_TIMEOUT))
    {
        EnetAppUtils_print("Failed to receive Event handle\r\n");
        EnetAppUtils_assert(false);
    }

    return recvdEventsMask;
}

static void EnetApp_handleEvent(const uint32_t eventMask)
{

    if (AppEventId_TXPKT & eventMask)
    {
        EnetApp_retrieveFreeTxPkts();
    }

    if (AppEventId_RXPKT & eventMask)
    {
        EnetApp_handleRxPkt();
    }

    if (AppEventId_PERIODIC_POLL & eventMask)
    {
        EnetApp_phyStateHandler();
    }

    if (gEnetLpbk.totalTxCnt < ENETLPBK_TEST_PKT_NUM)
    {
        EnetDma_PktQ TxPktQueue;
        EnetQueue_initQ(&TxPktQueue);
        EnetApp_preparePktQ(&TxPktQueue);
        gEnetLpbk.totalTxCnt += EnetApp_sendPktQ(&TxPktQueue);
    }

    return;
}

static int32_t  EnetApp_setMacAddress(Enet_Handle hEnet, uint8_t macAddr[ENET_MAC_ADDR_LEN], uint32_t macPort)
{
    int32_t  status = ENET_SOK;
    uint32_t coreId = EnetSoc_getCoreId();
    const Enet_Type enetType = hEnet->enetPer->enetType;

    /* Add port MAC entry in case of ICSSG dual MAC */
    if (ENET_ICSSG_DUALMAC == enetType)
    {
        Enet_IoctlPrms prms;
        IcssgMacPort_SetMacAddressInArgs inArgs;

        EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &macAddr[0U]);
        inArgs.macPort = macPort;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(hEnet, coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                    "Lwip2Enet_setMacAddress() failed ICSSG_MACPORT_IOCTL_ADD_INTERFACE_MACADDR: %d\r\n",
                    status);
        }
        EnetAppUtils_assert(status == ENET_SOK);
    }
    else if (ENET_ICSSG_SWITCH == enetType)
    {
        Enet_IoctlPrms prms;
        Icssg_MacAddr addr;

        /* Set host port's MAC address */
        EnetUtils_copyMacAddr(&addr.macAddr[0U], &macAddr[0U]);
        ENET_IOCTL_SET_IN_ARGS(&prms, &addr);
        {
            ENET_IOCTL(hEnet, coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms,
                    status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print(
                        "EnetAppUtils_addHostPortEntry() failed ICSSG_HOSTPORT_IOCTL_SET_MACADDR: %d\r\n",
                        status);
            }
            EnetAppUtils_assert(status == ENET_SOK);
        }
    }
    else
    {
        EnetAppUtils_assert(false);
    }
    return status;
}
