/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_intervlan_test.c
 *
 * \brief This file contains the cpsw_intervlan test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cpsw_intervlan_common.h"
#include "cpsw_test_intervlan.h"

#include "FreeRTOS.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void);

static uint32_t EnetApp_retrieveFreeTxPkts(void);

static void EnetApp_txTask(void *args);

static uint32_t EnetApp_receivePkts(void);

static void EnetApp_rxTask(void *args);

static void EnetApp_closeEnet(void);

static int32_t EnetApp_waitForLinkUp(void);

static void EnetApp_showCpswStats(void);

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                       EnetPhy_Mii *mii);

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii);

static void EnetApp_rxIsrFxn(void *appData);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(void);

static void EnetApp_initRxReadyPktQ(void);

static int32_t EnetApp_openDma(void);

static void EnetApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gEnetCpswInterVlanTaskStackTx[ENETCPSWINTERVLAN_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetCpswInterVlanTaskStackRx[ENETCPSWINTERVLAN_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern EnetCpswInterVlan_Obj gEnetCpswInterVlan;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_cpswInterVlanTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;

    if (gEnetCpswInterVlan.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    if (gEnetCpswInterVlan.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }

    EnetAppUtils_enableClocks(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gEnetCpswInterVlan.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetCpswInterVlan.rxDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetCpswInterVlan.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetCpswInterVlan.txDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gEnetCpswInterVlan.coreId = EnetSoc_getCoreId();
    EnetApp_driverInit();
    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gEnetCpswInterVlan.exitFlag     = false;
    gEnetCpswInterVlan.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId, &handleInfo);
    gEnetCpswInterVlan.hEnet = handleInfo.hEnet;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId, gEnetCpswInterVlan.coreId, &attachCoreOutArgs);
        gEnetCpswInterVlan.coreKey = attachCoreOutArgs.coreKey;
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

    if (status == ENET_SOK)
    {
        CpswAle_SetUcastEntryInArgs setUcastInArgs;
        uint32_t entryIdx;
        Enet_IoctlPrms prms;

        /* ALE entry with "secure" bit cleared is required for txsg */
        setUcastInArgs.addr.vlanId  = 0U;
        setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
        setUcastInArgs.info.blocked = false;
        setUcastInArgs.info.secure  = false;
        setUcastInArgs.info.super   = false;
        setUcastInArgs.info.ageable = false;
        setUcastInArgs.info.trunk   = false;
        EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], gEnetCpswInterVlan.hostMacAddr);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

        ENET_IOCTL(gEnetCpswInterVlan.hEnet, gEnetCpswInterVlan.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
        }

    }

    if (status == ENET_SOK)
    {
        status = EnetTestInterVlan_Run(&gEnetCpswInterVlan);
    }

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */
    if (status == ENET_SOK)
    {
        EnetApp_createRxTxTasks();

        SemaphoreP_pend(&gEnetCpswInterVlan.txDoneSemObj, SystemP_WAIT_FOREVER);
        SemaphoreP_pend(&gEnetCpswInterVlan.rxDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Print network statistics */
    if (status == ENET_SOK)
    {
        if (Enet_isCpswFamily(gEnetCpswInterVlan.enetType))
        {
            EnetApp_showCpswStats();
        }
    }

    /* Print DMA statistics */
    if (status == ENET_SOK)
    {
        EnetAppUtils_showRxChStats(gEnetCpswInterVlan.hRxCh);
        EnetAppUtils_showTxChStats(gEnetCpswInterVlan.hTxCh);
    }

    /* Close Enet DMA driver */
    EnetApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId, gEnetCpswInterVlan.coreId, gEnetCpswInterVlan.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId);
    gEnetCpswInterVlan.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gEnetCpswInterVlan.enetType, gEnetCpswInterVlan.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gEnetCpswInterVlan.rxSemObj);
    SemaphoreP_destruct(&gEnetCpswInterVlan.rxDoneSemObj);
    SemaphoreP_destruct(&gEnetCpswInterVlan.txSemObj);
    SemaphoreP_destruct(&gEnetCpswInterVlan.txDoneSemObj);

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 3U;
    taskParams.stack          = gEnetCpswInterVlanTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetCpswInterVlanTaskStackRx);
    taskParams.args           = (void*)&gEnetCpswInterVlan.rxSemObj;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&gEnetCpswInterVlan.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gEnetCpswInterVlanTaskStackTx;
    taskParams.stackSize      = sizeof(gEnetCpswInterVlanTaskStackTx);
    taskParams.args           = (void*)&gEnetCpswInterVlan.txSemObj;
    taskParams.name           = "Tx Task";
    taskParams.taskMain       = &EnetApp_txTask;

    status = TaskP_construct(&gEnetCpswInterVlan.txTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetCpswInterVlan.hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gEnetCpswInterVlan.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return txFreeQCnt;
}

static void EnetApp_txTask(void *args)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    EthVlanFrame *frame;
    uint32_t txRetrievePktCnt;
    uint32_t pktCnt;
    int32_t status = ENET_SOK;
    extern uint8_t testDstMcastMacAddr[ENET_MAC_ADDR_LEN];
    extern const uint16_t hostTxVlanId;

    gEnetCpswInterVlan.totalTxCnt = 0U;
    pktCnt = 0U;
    while (pktCnt < ENETCPSWINTERVLAN_TEST_PKT_NUM)
    {
        /* Transmit a single packet */
        EnetQueue_initQ(&txSubmitQ);

        /* Dequeue one free TX Eth packet */
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetCpswInterVlan.txFreePktInfoQ);

        while (NULL != pktInfo)
        {
            uint16_t tci;
            const uint16_t pcp = 0;

            pktCnt++;
            frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
            memcpy(frame->hdr.dstMac, testDstMcastMacAddr, ENET_MAC_ADDR_LEN);
            memcpy(frame->hdr.srcMac, &gEnetCpswInterVlan.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
            frame->hdr.tpid = Enet_htons(ETHERTYPE_VLAN_TAG);
            tci = hostTxVlanId & 0xFFFU;
            tci |= (pcp & 0x7) << 13;
            frame->hdr.tci  = Enet_htons(tci);
            frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
            memset(&frame->payload[0U], (uint8_t)(0xA5 + pktCnt), ENETCPSWINTERVLAN_TEST_PKT_LEN);

            pktInfo->sgList.list[0].segmentFilledLen = ENETCPSWINTERVLAN_TEST_PKT_LEN + sizeof(EthVlanFrameHeader);
            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->appPriv    = &gEnetCpswInterVlan;
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue the packet for later transmission */
            EnetQueue_enq(&txSubmitQ, &pktInfo->node);

            if (pktCnt >= ENETCPSWINTERVLAN_TEST_PKT_NUM)
            {
                break;
            }

            /* Dequeue one free TX Eth packet */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetCpswInterVlan.txFreePktInfoQ);
        }

        while (0U != EnetQueue_getQCount(&txSubmitQ))
        {
            uint32_t txCnt = EnetQueue_getQCount(&txSubmitQ);
            status = EnetDma_submitTxPktQ(gEnetCpswInterVlan.hTxCh,
                                          &txSubmitQ);
            SemaphoreP_pend(&gEnetCpswInterVlan.txSemObj, SystemP_WAIT_FOREVER);

            /* Retrieve TX free packets */
            if (status == ENET_SOK)
            {
                txCnt            = txCnt - EnetQueue_getQCount(&txSubmitQ);
                txRetrievePktCnt = 0U;
                while (txRetrievePktCnt != txCnt)
                {
                    /* This is not failure as HW is busy sending packets, we
                     * need to wait and again call retrieve packets */
                    ClockP_usleep(1000);
                    txRetrievePktCnt += EnetApp_retrieveFreeTxPkts();
                }
            }
            else
            {
                break;
            }
        }
    }

    gEnetCpswInterVlan.totalTxCnt += pktCnt;

    EnetAppUtils_print("Transmitted %d packets \r\n", gEnetCpswInterVlan.totalTxCnt);
    SemaphoreP_post(&gEnetCpswInterVlan.txDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_txTask() and exit..\r\n");
    TaskP_destruct(&gEnetCpswInterVlan.txTaskObj);
    TaskP_exit();
}

static uint32_t EnetApp_receivePkts(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetCpswInterVlan.hRxCh, &rxReadyQ);
    if (status == ENET_SOK)
    {
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);

        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ
        **/
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(&gEnetCpswInterVlan.rxReadyQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("receivePkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return rxReadyCnt;
}

static void EnetApp_rxTask(void *args)
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t rxReadyCnt;
    uint32_t loopRxPktCnt;
    int32_t status = ENET_SOK;

    gEnetCpswInterVlan.totalRxCnt = 0U;

    loopRxPktCnt = 0U;
    /* Wait for packet reception */
    do
    {
        SemaphoreP_pend(&gEnetCpswInterVlan.rxSemObj, SystemP_WAIT_FOREVER);
        /* Get the packets received so far */
        rxReadyCnt = EnetApp_receivePkts();
        if (rxReadyCnt > 0U)
        {
            /* Consume the received packets and release them */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetCpswInterVlan.rxReadyQ);
            while (NULL != pktInfo)
            {
                EnetDma_checkPktState(&pktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_READYQ,
                                      ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Consume the packet by just printing its content */
                if (gEnetCpswInterVlan.printFrame)
                {
                    uint32_t packetPrintLen;

                    frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                    packetPrintLen = pktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);

                    EnetAppUtils_printFrame(frame,
                                            packetPrintLen);
                }

                /* Release the received packet */
                EnetQueue_enq(&gEnetCpswInterVlan.rxFreeQ, &pktInfo->node);
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetCpswInterVlan.rxReadyQ);
            }

            /*Submit now processed buffers */
            if (status == ENET_SOK)
            {
                EnetAppUtils_validatePacketState(&gEnetCpswInterVlan.rxFreeQ,
                                                 ENET_PKTSTATE_APP_WITH_FREEQ,
                                                 ENET_PKTSTATE_APP_WITH_DRIVER);

                EnetDma_submitRxPktQ(gEnetCpswInterVlan.hRxCh,
                                     &gEnetCpswInterVlan.rxFreeQ);
            }
        }

        loopRxPktCnt += rxReadyCnt;
    }
    while (loopRxPktCnt < ENETCPSWINTERVLAN_TEST_PKT_NUM);

    gEnetCpswInterVlan.totalRxCnt += loopRxPktCnt;

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", ENETCPSWINTERVLAN_TEST_PKT_NUM, gEnetCpswInterVlan.totalRxCnt);
    }
    else
    {
        EnetAppUtils_print("Received %d packets\r\n", gEnetCpswInterVlan.totalRxCnt);
    }

    SemaphoreP_post(&gEnetCpswInterVlan.rxDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_rxTask() and exit..\r\n");
    TaskP_destruct(&gEnetCpswInterVlan.rxTaskObj);
    TaskP_exit();
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    EnetTestInterVlan_setOpenPrms(&gEnetCpswInterVlan, cpswCfg);
}

void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    CpswMacPort_Cfg *cpswMacCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;
    EnetPhy_Mii phyMii;
    const EnetBoard_PhyCfg *boardPhyCfg = NULL;

    /* Setup board for requested Ethernet port */
    ethPort.enetType = gEnetCpswInterVlan.enetType;
    ethPort.instId   = gEnetCpswInterVlan.instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = gEnetCpswInterVlan.boardId;
    EnetApp_macMode2MacMii(gEnetCpswInterVlan.macMode, &ethPort.mii);

    status = EnetBoard_setupPorts(&ethPort, 1U);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Set port link params */
    linkArgs->macPort = macPort;

    cpswMacCfg = linkArgs->macCfg;

    EnetApp_macMode2MacMii(gEnetCpswInterVlan.macMode, mii);

    /* Set PHY configuration params */
    EnetPhy_initCfg(phyCfg);
    status = EnetApp_macMode2PhyMii(gEnetCpswInterVlan.macMode, &phyMii);

    if (status == ENET_SOK)
    {
        boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
        if (boardPhyCfg != NULL)
        {
            phyCfg->phyAddr     = boardPhyCfg->phyAddr;
            phyCfg->isStrapped  = boardPhyCfg->isStrapped;
            phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
            phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
            memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);
        }
        else
        {
            EnetAppUtils_print("PHY info not found\r\n");
            EnetAppUtils_assert(false);
        }

        if ((phyMii == ENETPHY_MAC_MII_MII) ||
            (phyMii == ENETPHY_MAC_MII_RMII))
        {
            linkCfg->speed = ENET_SPEED_100MBIT;
        }
        else
        {
            linkCfg->speed = (ENETCPSWINTERVLAN_TEST_LOOPBACK_EN == true) ? ENET_SPEED_1GBIT : ENET_SPEED_AUTO;
        }

        linkCfg->duplexity = (ENETCPSWINTERVLAN_TEST_LOOPBACK_EN == true) ? ENET_DUPLEX_FULL : ENET_DUPLEX_AUTO;
    }

    /* MAC and PHY cpsw_intervlans are mutually exclusive */
    phyCfg->loopbackEn = ENETCPSWINTERVLAN_TEST_LOOPBACK_EN;
    cpswMacCfg->loopbackEn = false;
    EnetTestInterVlan_updatePortLinkCfg(linkArgs, macPort);
}

static void EnetApp_closeEnet(void)
{
    Enet_IoctlPrms prms;
    int32_t status;
    uint32_t i;
    
    for (i = 0; i < gEnetCpswInterVlan.numMacPorts; i++)
    {

        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &gEnetCpswInterVlan.macPortList[i]);

        ENET_IOCTL(gEnetCpswInterVlan.hEnet, gEnetCpswInterVlan.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to close port link: %d\r\n", status);
        }

    }
    /* Close Enet driver */
    Enet_close(gEnetCpswInterVlan.hEnet);

    gEnetCpswInterVlan.hEnet = NULL;
}



static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;
    uint32_t i;
    
    
    for (i = 0; i < gEnetCpswInterVlan.numMacPorts; i++)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetCpswInterVlan.macPortList[i], &linked);

        while (!linked)
        {
            ENET_IOCTL(gEnetCpswInterVlan.hEnet, gEnetCpswInterVlan.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                                ENET_MACPORT_ID(gEnetCpswInterVlan.macPortList[i]), status);
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

static void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gEnetCpswInterVlan.hEnet, gEnetCpswInterVlan.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g *)&portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        uint32_t i;

        for (i = 0; i < gEnetCpswInterVlan.numMacPorts; i++)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetCpswInterVlan.macPortList[i], &portStats);
            ENET_IOCTL(gEnetCpswInterVlan.hEnet, gEnetCpswInterVlan.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status == ENET_SOK)
            {
                EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
                EnetAppUtils_print("-----------------------------------------\r\n");
                EnetAppUtils_printMacPortStats2G((CpswStats_MacPort_2g *)&portStats);
                EnetAppUtils_print("\r\n");
            }
            else
            {
                EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
            }
        }
    }
}

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
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
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
            break;

        case RGMII:
            mii->layerType    = ENET_MAC_LAYER_GMII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_FORCED;
            break;
        default:
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }
}

static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetCpswInterVlan.rxSemObj);
}

static void EnetApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetCpswInterVlan.txSemObj);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetCpswInterVlan.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetCpswInterVlan,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetCpswInterVlan.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetCpswInterVlan.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    EnetQueue_initQ(&gEnetCpswInterVlan.rxFreeQ);
    EnetQueue_initQ(&gEnetCpswInterVlan.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetCpswInterVlan,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gEnetCpswInterVlan.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetCpswInterVlan.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gEnetCpswInterVlan.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gEnetCpswInterVlan.hRxCh,
                         &gEnetCpswInterVlan.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetCpswInterVlan.rxFreeQ));
}

static int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo; 

        txInArgs.cbArg   = &gEnetCpswInterVlan;
        txInArgs.notifyCb = EnetApp_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gEnetCpswInterVlan.txChNum = txChInfo.txChNum;
        gEnetCpswInterVlan.hTxCh   = txChInfo.hTxCh;


        EnetApp_initTxFreePktQ();

        if (NULL != gEnetCpswInterVlan.hTxCh)
        {
            status = ENET_SOK;
            if (ENET_SOK != status)
            {
                EnetAppUtils_print("EnetUdma_startTxCh() failed: %d\r\n", status);
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

    /* Open the CPSW RX flow  */
    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        EnetApp_GetDmaHandleInArgs     rxInArgs;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetCpswInterVlan;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gEnetCpswInterVlan.rxChNum = rxChInfo.rxChNum;
        gEnetCpswInterVlan.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetCpswInterVlan.hostMacAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        if (NULL == gEnetCpswInterVlan.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gEnetCpswInterVlan.hRxCh);
        }
        else
        {
            /* Submit all ready RX buffers to DMA.*/
            EnetApp_initRxReadyPktQ();
        }
    }

    return status;
}

static void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetCpswInterVlan.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetCpswInterVlan.hEnet,
                       gEnetCpswInterVlan.coreKey,
                       gEnetCpswInterVlan.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetCpswInterVlan.hEnet,
                       gEnetCpswInterVlan.coreKey,
                       gEnetCpswInterVlan.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetCpswInterVlan.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gEnetCpswInterVlan.txFreePktInfoQ);

}

