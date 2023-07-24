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
 * \file  testcase_singlepkt_tc1_icssg.c
 *
 * \brief This file contains the loopback test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "testcase_common_cfg_icssg.h"
#include <kernel/dpl/EventP.h>

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

static uint32_t TestApp_retrieveFreeTxPkts(void);

static void TestApp_receivePkts(EnetDma_RxChHandle hRx, EnetDma_PktQ* pRecvdPktQs);

static int32_t TestApp_waitForLinkUp(void);

static int32_t TestApp_macMode2PhyMii(emac_mode macMode, EnetPhy_Mii *mii);

static void TestApp_macMode2MacMii(emac_mode macMode,
                                   EnetMacPort_Interface *mii);

static void TestApp_initTxFreePktQ(void);

static void TestApp_initRxReadyPktQ(void);

static int32_t TestApp_openDma(void);

static void TestApp_closeDma(void);

static uint32_t TestApp_receiveEvents(EventP_Object *pEvent);

static void TestApp_handleEvent(const uint32_t eventMask);

static void TestApp_preparePktQ(EnetDma_PktQ *pPktQ);

static uint32_t TestApp_sendPktQ(EnetDma_PktQ *pPktQ);

static void TestApp_postTxEvent(void *pArg);

static void TestApp_postRxEvent(void *pArg);

static int32_t  TestApp_setMacAddress(Enet_Handle hEnet, uint8_t macAddr[ENET_MAC_ADDR_LEN], uint32_t macPort);

void TestApp_printStats();
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
//static uint8_t gTestAppTaskStackTx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
//static uint8_t gTestAppTaskStackRx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern TestApp_Obj gTestApp;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TestApp_loopbackTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    if (gTestApp.enetType == ENET_ICSSG_SWITCH)
    {
        EnetAppUtils_print("ICSSG_SWITCH Test\r\n");
    }
    if (gTestApp.enetType == ENET_ICSSG_DUALMAC)
    {
        EnetAppUtils_print("ICSSG_DUALMAC Test\r\n");
    }

    EnetApp_driverInit();

    EnetAppUtils_enableClocks(gTestApp.enetType, gTestApp.instId);

    /* Create Global Event Object */
    status = EventP_construct(&gTestApp.appEvents);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gTestApp.coreId = EnetSoc_getCoreId();
    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gTestApp.enetType, gTestApp.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gTestApp.exitFlag = false;
    gTestApp.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gTestApp.enetType, gTestApp.instId,
                              &handleInfo);
    gTestApp.hEnet = handleInfo.hEnet;
    gTestApp.hUdmaDrv = handleInfo.hUdmaDrv;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gTestApp.enetType, gTestApp.instId,
                           gTestApp.coreId, &attachCoreOutArgs);
        gTestApp.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = TestApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    TestApp_printStats();
    /* Wait for link up */
    EnetApp_initPhyStateHandlerTask(&gTestApp.appEvents);
    TestApp_setMacAddress(gTestApp.hEnet, gTestApp.hostMacAddr, gTestApp.macPortList[0]);

    if ((status == ENET_SOK)
            && (gTestApp.testLoopBackType == LOOPBACK_TYPE_PHY))
    {
        status = TestApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */

    TestApp_postTxEvent((void*) &gTestApp.appEvents);
    gTestApp.totalTxCnt = 0;
    gTestApp.totalRxCnt = 0;
    while (true)
    {
        const uint32_t recvdEventsMask = TestApp_receiveEvents(&gTestApp.appEvents);

        if (recvdEventsMask != AppEventId_NONE)
        {
            TestApp_handleEvent(recvdEventsMask);
            if (gTestApp.totalRxCnt >= ENETLPBK_TEST_PKT_NUM)
            {
                EnetAppUtils_print("completed\r\n");
                break;
            }
        }
    }

    TestApp_retrieveFreeTxPkts();
    if (status == ENET_SOK)
    {
        TestApp_printStats();
    }

    /* Close Enet DMA driver */
    TestApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gTestApp.enetType, gTestApp.instId, gTestApp.coreId,
                       gTestApp.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gTestApp.enetType, gTestApp.instId);
    gTestApp.hEnet = NULL;

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gTestApp.enetType, gTestApp.instId);

    /* Delete all Global Event */
    EventP_destruct(&gTestApp.appEvents);

    EnetAppUtils_print("Test complete: %s\r\n",
                       (status == ENET_SOK) ? "PASS" : "FAIL");


    return status;
}

void TestApp_printStats()
{
    /* Statistics */
    IcssgStats_MacPort gEnetMp_icssgStats;
    IcssgStats_Pa gEnetMp_icssgPaStats;
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    if (Enet_isIcssFamily(gTestApp.enetType))
    {
        EnetAppUtils_print("\n - HOST PORT statistics\r\n");
        EnetAppUtils_print("--------------------------------\r\n");
        ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetMp_icssgPaStats);
        ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
                   ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get PA stats\r\n");
        }

        EnetAppUtils_printIcssgPaStats(&gEnetMp_icssgPaStats);
        EnetAppUtils_print("\r\n");

        for (uint32_t portIdx = 0; portIdx < gTestApp.numMacPorts; portIdx++)
        {
            macPort = (ENET_ICSSG_DUALMAC == gTestApp.enetType)? ENET_MAC_PORT_1 : gTestApp.macPortList[portIdx];
            EnetAppUtils_print("\n Mac %d statistics\r\n", portIdx + 1);
            EnetAppUtils_print("--------------------------------\r\n");
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetMp_icssgStats);

            ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u stats\r\n", ENET_MACPORT_ID(macPort));
            }

            EnetAppUtils_printIcssgMacPortStats(&gEnetMp_icssgStats, false);
            EnetAppUtils_print("\r\n");
        }
    }
}

void TestApp_resetstats()
{

    /* Statistics */
    //IcssgStats_MacPort gEnetMp_icssgStats;
    //IcssgStats_Pa gEnetMp_icssgPaStats;
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    EnetAppUtils_print("Reset statistics\r\n");

    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
               ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset  host port stats\r\n");
    }

    macPort = ENET_MAC_PORT_1;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
               ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n",
                           ENET_MACPORT_ID(macPort));
    }

    ClockP_usleep(50000);

    macPort = ENET_MAC_PORT_2;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
               ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset port %u stats\r\n",
                           ENET_MACPORT_ID(macPort));
    }

    ClockP_usleep(50000);

    macPort = ENET_MAC_PORT_3;

    ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
    ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
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

static void TestApp_postRxEvent(void *pArg)
{

    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_RXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post Rx Event\r\n");
        EnetAppUtils_assert(false);
    }

}

static void TestApp_postTxEvent(void *pArg)
{
    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_TXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post TX Event\r\n");
        EnetAppUtils_assert(false);
    }
}

void TestApp_postPollEvent(void *pArg)
{
    EventP_Object *pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_PERIODIC_POLL);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post Poll Event\r\n");
        EnetAppUtils_assert(false);
    }
}

static uint32_t TestApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any ICSSG packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gTestApp.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pktInfo->node);
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

static void TestApp_preparePktQ(EnetDma_PktQ *pktQueue)
{

    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    int32_t status = ENET_SOK;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = { 0xffU, 0xffU, 0xffU, 0xffU, 0xffU,
                                             0xffU };

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gTestApp.txFreePktInfoQ);

    uint32_t totalPktCnt = gTestApp.totalTxCnt;
    if (NULL != pktInfo)
    {
        /* Fill the TX Eth frame with test content */
        frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;
        memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
        memcpy(frame->hdr.srcMac, &gTestApp.hostMacAddr[0U],
               ENET_MAC_ADDR_LEN);
        frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
        memset(&frame->payload[0U], (uint8_t) (0xA5 + totalPktCnt),
               ENETLPBK_TEST_PKT_LEN);
        totalPktCnt++;
        pktInfo->sgList.list[0].segmentFilledLen = ENETLPBK_TEST_PKT_LEN
                + sizeof(EthFrameHeader);
        pktInfo->sgList.numScatterSegments = 1;
        pktInfo->chkSumInfo = 0U;
        pktInfo->appPriv = &gTestApp;
        pktInfo->txPortNum = ENET_MAC_PORT_INV;
        EnetDma_checkPktState(&pktInfo->pktState, ENET_PKTSTATE_MODULE_APP,
                              ENET_PKTSTATE_APP_WITH_FREEQ,
                              ENET_PKTSTATE_APP_WITH_DRIVER);

        status = EnetDma_submitTxPkt(gTestApp.hTxCh, pktInfo);
        ClockP_usleep(1000);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("EnetDma_submitTxPkt() failed to submit pkts: %d\r\n",
                           status);
        }
        gTestApp.totalTxCnt++;
        /* Enqueue the packet for later transmission */
        //EnetQueue_enq(pktQueue, &pktInfo->node);
    }

//    EnetAppUtils_print("PrepareTx : %d\r\n", EnetQueue_getQCount(pktQueue));

    //return EnetQueue_getQCount(pktQueue);
}

static void TestApp_handleRxPkt()
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    int32_t status = ENET_SOK;

    /* Get the packets received so far */
    for (uint32_t chIdx = 0; chIdx < ENET_SYSCFG_RX_FLOWS_NUM; chIdx++)
    {
        status = EnetDma_retrieveRxPkt(gTestApp.hRxCh[chIdx], &pktInfo);
        //TestApp_receivePkts(gTestApp.hRxCh[chIdx], &gTestApp.rxReadyQ);

        //if (EnetQueue_getQCount(&gTestApp.rxReadyQ) > 0U)
        {
            /* Consume the received packets and release them */
            //pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gTestApp.rxReadyQ);
            if (NULL != pktInfo)
            {
                EnetDma_checkPktState(&pktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_READYQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ);

                frame = (EthFrame*) pktInfo->sgList.list[0].bufPtr;

                if (memcmp(frame->hdr.srcMac, gTestApp.hostMacAddr, ENET_MAC_ADDR_LEN) == 0U)
                {
                    gTestApp.totalRxCnt++;
                }
                /* Consume the packet by just printing its content */
                if (gTestApp.printFrame)
                {
                    uint32_t packetPrintLen;

                    packetPrintLen = pktInfo->sgList.list[0].segmentFilledLen
                            - sizeof(EthFrameHeader);
                    EnetAppUtils_printFrame(frame, packetPrintLen);
                }

                EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);
                /* Release the received packet */
                EnetDma_submitRxPkt(gTestApp.hRxCh[chIdx], pktInfo);
                //EnetQueue_enq(&gTestApp.rxFreeQ, &pktInfo->node);
                //pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gTestApp.rxReadyQ);
            }

            /*Submit now processed buffers */
            /*if (status == ENET_SOK)
            {
                EnetAppUtils_validatePacketState(&gTestApp.rxFreeQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ,
                        ENET_PKTSTATE_APP_WITH_DRIVER);

                EnetDma_submitRxPktQ(gTestApp.hRxCh[chIdx],
                        &gTestApp.rxFreeQ);
            }*/
        }
        if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", ENETLPBK_TEST_PKT_NUM, gTestApp.totalRxCnt);
    }
    else
    {
        //EnetAppUtils_print("Received %d packets\r\n", gTestApp.totalRxCnt);
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
    EnetAppUtils_print("\nInit configs EnetType:%u, InstId :%u\r\n", enetType,
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

void EnetApp_initLinkArgs(Enet_Type enetType, uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs, Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;
    EnetPhy_Mii phyMii;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gTestApp.enetType;
    ethPort.instId = gTestApp.instId;
    ethPort.macPort = gTestApp.macPortList[0];
    ethPort.boardId = gTestApp.boardId;
    TestApp_macMode2MacMii(gTestApp.macMode, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Set port link params */
    linkArgs->macPort = macPort;
    IcssgMacPort_Cfg* pIcssgMacCfg = linkArgs->macCfg;
    IcssgMacPort_initCfg(pIcssgMacCfg);
    pIcssgMacCfg->specialFramePrio = 1U;
    TestApp_macMode2MacMii(gTestApp.macMode, mii);


    if (gTestApp.testLoopBackType == LOOPBACK_TYPE_PHY)
    {
        const EnetBoard_PhyCfg *boardPhyCfg = NULL;

        /* Set PHY configuration params */
        EnetPhy_initCfg(phyCfg);
        status = TestApp_macMode2PhyMii(gTestApp.macMode, &phyMii);

        if (status == ENET_SOK)
        {
            boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
            if (boardPhyCfg != NULL)
            {

                EnetPhy_initCfg(phyCfg);

                if ((ENET_ICSSG_DUALMAC == gTestApp.enetType)
                        && (2U == gTestApp.instId))
                {
                    phyCfg->phyAddr = CONFIG_ENET_ICSS0_PHY1_ADDR;
                }
                else if ((ENET_ICSSG_DUALMAC == gTestApp.enetType)
                        && (3U == gTestApp.instId))
                {
                    phyCfg->phyAddr = CONFIG_ENET_ICSS0_PHY2_ADDR;

                }
                else
                {
                    phyCfg->phyAddr = boardPhyCfg->phyAddr;
                }
                //phyCfg->phyAddr     = boardPhyCfg->phyAddr;
                phyCfg->isStrapped = boardPhyCfg->isStrapped;
                phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
                phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
                memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg,
                       phyCfg->extendedCfgSize);
            }
            else
            {
                EnetAppUtils_print("PHY info not found\r\n");
                EnetAppUtils_assert(false);
            }

            if ((phyMii == ENETPHY_MAC_MII_MII)
                    || (phyMii == ENETPHY_MAC_MII_RMII))
            {
                linkCfg->speed = ENET_SPEED_100MBIT;
            }
            else
            {
                linkCfg->speed = ENET_SPEED_100MBIT;
            }

            linkCfg->duplexity = ENET_DUPLEX_FULL;
        }
        phyCfg->loopbackEn = true;
    }
    else
    {
        EnetAppUtils_assert(false);
    }

}

static int32_t TestApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    for (uint32_t portIdx = 0; portIdx < gTestApp.numMacPorts; portIdx++)
    {
        bool linked = false;
        while (!linked)
        {
            EnetApp_phyStateHandler();
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &gTestApp.macPortList[portIdx], &linked);

            ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
                       ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                                   ENET_MACPORT_ID(gTestApp.macPortList[portIdx]), status);
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

static int32_t TestApp_macMode2PhyMii(emac_mode macMode, EnetPhy_Mii *mii)
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

static void TestApp_macMode2MacMii(emac_mode macMode,
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

static void TestApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;

    /* Initialize all queues */
    EnetQueue_initQ(&gTestApp.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

        pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);

        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                     ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gTestApp.txFreePktInfoQ));
}

static void TestApp_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;

    EnetQueue_initQ(&gTestApp.rxFreeQ);
    EnetQueue_initQ(&gTestApp.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (uint32_t chIdx = 0; chIdx < ENET_SYSCFG_RX_FLOWS_NUM; chIdx++)
    {
        /* Divide the total packets available in pool equally between to Rx DMA channels */
        for (uint32_t pktCount = 0U; pktCount < (ENET_SYSCFG_TOTAL_NUM_RX_PKT / 2); pktCount++)
        {
            uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

            pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                           ENETDMA_CACHELINE_ALIGNMENT,
                                           ENET_ARRAYSIZE(scatterSegments),
                                           scatterSegments);

            EnetAppUtils_assert(pPktInfo != NULL);
            ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                         ENET_PKTSTATE_APP_WITH_FREEQ);
            EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);

            /* Retrieve any CPSW packets which are ready */
            status = EnetDma_retrieveRxPktQ(gTestApp.hRxCh[chIdx], &rxReadyQ);
            EnetAppUtils_assert(status == ENET_SOK);
            /* There should not be any packet with DMA during init */
            EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

            EnetAppUtils_validatePacketState(&gTestApp.rxFreeQ,
                                             ENET_PKTSTATE_APP_WITH_FREEQ,
                                             ENET_PKTSTATE_APP_WITH_DRIVER);

            EnetDma_submitRxPktQ(gTestApp.hRxCh[chIdx], &gTestApp.rxFreeQ);
            EnetAppUtils_assert(0U == EnetQueue_getQCount(&gTestApp.rxFreeQ));
        }
    }
}

static int32_t TestApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs txInArgs;
        EnetApp_GetTxDmaHandleOutArgs txChInfo;

        txInArgs.cbArg = &gTestApp.appEvents;
        txInArgs.notifyCb = TestApp_postTxEvent;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0, &txInArgs, &txChInfo);

        gTestApp.txChNum = txChInfo.txChNum;
        gTestApp.hTxCh = txChInfo.hTxCh;

        TestApp_initTxFreePktQ();

        if (NULL != gTestApp.hTxCh)
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

        rxInArgs.notifyCb = TestApp_postRxEvent;
        rxInArgs.cbArg = &gTestApp.appEvents;

        /* Open First Rx Channel */
        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0, &rxInArgs, &rxChInfo);

        if (NULL == rxChInfo.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n", ENET_DMA_RX_CH0);
            EnetAppUtils_assert(false);
        }
        gTestApp.hRxCh[ENET_DMA_RX_CH0] = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress > 0);
        EnetUtils_copyMacAddr(gTestApp.hostMacAddr, rxChInfo.macAddr[0]);

#ifdef ENET_DMA_RX_CH1
        /* Open Second Rx Channel */
        rxInArgs.notifyCb = TestApp_postRxEvent;
        rxInArgs.cbArg = &gTestApp.appEvents;
        TestApp_getRxDmaHandle(ENET_DMA_RX_CH1, &rxInArgs, &rxChInfo);
        if (NULL == rxChInfo.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n", ENET_DMA_RX_CH1);
            EnetAppUtils_assert(false);
        }
        gTestApp.hRxCh[ENET_DMA_RX_CH1] = rxChInfo.hRxCh;
#endif
        /* Submit all ready RX buffers to DMA.*/
        TestApp_initRxReadyPktQ();
    }

    return status;
}

static void TestApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;
    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gTestApp.rxReadyQ));

    /* Close RX channel - ENET_DMA_RX_CH0*/
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma(ENET_DMA_RX_CH0, gTestApp.hEnet, gTestApp.coreKey,
                       gTestApp.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close RX channel - ENET_DMA_RX_CH1*/
#ifdef ENET_DMA_RX_CH1
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    TestApp_closeRxDma(ENET_DMA_RX_CH1, gTestApp.hEnet, gTestApp.coreKey,
                       gTestApp.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
#endif

    /* Close TX channel - ENET_DMA_TX_CH0 */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0, gTestApp.hEnet, gTestApp.coreKey,
                       gTestApp.coreId, &fqPktInfoQ, &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gTestApp.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gTestApp.txFreePktInfoQ);

}

static uint32_t TestApp_receiveEvents(EventP_Object *pEvent)
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

static void TestApp_handleEvent(const uint32_t eventMask)
{

    if (AppEventId_TXPKT & eventMask)
    {
        TestApp_retrieveFreeTxPkts();
    }

    if (AppEventId_RXPKT & eventMask)
    {
        TestApp_handleRxPkt();
    }

    if (AppEventId_PERIODIC_POLL & eventMask)
    {
        EnetApp_phyStateHandler();
    }

    if (gTestApp.totalTxCnt < ENETLPBK_TEST_PKT_NUM)
    {
        EnetDma_PktQ TxPktQueue;
        EnetQueue_initQ(&TxPktQueue);
        TestApp_preparePktQ(&TxPktQueue);
        //gTestApp.totalTxCnt += TestApp_sendPktQ(&TxPktQueue);
    }

    return;
}

static int32_t  TestApp_setMacAddress(Enet_Handle hEnet, uint8_t macAddr[ENET_MAC_ADDR_LEN], uint32_t macPort)
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
