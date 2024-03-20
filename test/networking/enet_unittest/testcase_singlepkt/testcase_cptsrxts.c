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
 * \file  testcase_cptsrxts.c
 *
 * \brief This file contains the loopback test for cpts rx timestamps.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "testcase_common_cfg.h"
#include "testcase_cptsrxts.h"
#include "enet_mod_timesync.h"
#include "FreeRTOS.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Value of seconds in nanoseconds. Useful for calculations*/
#define TIME_SEC_TO_NS                           (1000000000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void TestApp_createRxTxTasks(void);

static uint32_t TestApp_retrieveFreeTxPkts(void);

static void TestApp_txTask(void *args);

static uint32_t TestApp_receivePkts(void);

static void TestApp_rxTask(void *args);

static void TestApp_rxIsrFxn(void *appData);

static void TestApp_txIsrFxn(void *appData);

static void TestApp_initTxFreePktQ(void);

static void TestApp_initRxReadyPktQ(void);

static int32_t TestApp_openDma(void);

static void TestApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gTestAppTaskStackTx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gTestAppTaskStackRx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern TestApp_Obj gTestApp;

uint8_t gptpPktPayload[] =
{ 0x10, 0x12, 0x00, 0x2c, 0x00, 0x00, 0x02, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xf4, 0x84, 0x4c, 0xff,
  0xfe, 0xfb, 0xc0, 0x42, 0x00, 0x01, 0x00, 0x00,
  0x00, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void TestApp_setPortTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg)
{
    tsPortEventCfg->commonPortIpCfg.ttlNonzeroEn = true;
    tsPortEventCfg->commonPortIpCfg.tsIp107En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp129En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp130En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp131En = false;
    tsPortEventCfg->commonPortIpCfg.tsIp132En = false;
    tsPortEventCfg->commonPortIpCfg.tsPort319En = true;
    tsPortEventCfg->commonPortIpCfg.tsPort320En = true;
    tsPortEventCfg->commonPortIpCfg.unicastEn = false;
    tsPortEventCfg->domainOffset = 4U;
    tsPortEventCfg->ltype2En = false;
    tsPortEventCfg->rxAnnexDEn = true;
    tsPortEventCfg->rxAnnexEEn = true;
    tsPortEventCfg->rxAnnexFEn = true;
    tsPortEventCfg->txAnnexDEn = true;
    tsPortEventCfg->txAnnexEEn = true;
    tsPortEventCfg->txAnnexFEn = true;
    tsPortEventCfg->txHostTsEn = true;
    tsPortEventCfg->mcastType = 0U;
    tsPortEventCfg->messageType = 0xFFFFU;
    tsPortEventCfg->seqIdOffset = 30U;
    /* VLAN untagged */
    tsPortEventCfg->rxVlanType = ENET_MACPORT_VLAN_TYPE_SINGLE_TAG;
    tsPortEventCfg->txVlanType = ENET_MACPORT_VLAN_TYPE_SINGLE_TAG;
    tsPortEventCfg->vlanLType1 = 0x8100U;
    tsPortEventCfg->vlanLType2 = 0U;

}

int32_t TestApp_CptsRxTsTestcase(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;
    CpswMacPort_EnableTsEventInArgs enableTsEventInArgs;
    Enet_IoctlPrms prms;

    if (gTestApp.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    if (gTestApp.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }

    EnetAppUtils_enableClocks(gTestApp.enetType, gTestApp.instId);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gTestApp.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gTestApp.rxDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gTestApp.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gTestApp.txDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gTestApp.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gTestApp.enetType, gTestApp.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gTestApp.exitFlag     = false;
    gTestApp.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gTestApp.enetType, gTestApp.instId, &handleInfo);
    gTestApp.hEnet = handleInfo.hEnet;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gTestApp.enetType, gTestApp.instId, gTestApp.coreId, &attachCoreOutArgs);
        gTestApp.coreKey = attachCoreOutArgs.coreKey;
    }

    TestApp_setPortTsEventPrms(&enableTsEventInArgs.tsEventCfg);
    enableTsEventInArgs.macPort = gTestApp.macPort;
    ENET_IOCTL_SET_IN_ARGS(&prms, &enableTsEventInArgs);
	ENET_IOCTL(gTestApp.hEnet,
			   gTestApp.coreId,
			   CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT,
			   &prms,
			   status);
	EnetAppUtils_assert(status == ENET_SOK);

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = TestApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        CpswAle_SetUcastEntryInArgs setUcastInArgs;
        uint32_t entryIdx;
        /* ALE entry with "secure" bit cleared is required for loopback */
        setUcastInArgs.addr.vlanId  = 0U;
        setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
        setUcastInArgs.info.blocked = false;
        setUcastInArgs.info.secure  = false;
        setUcastInArgs.info.super   = false;
        setUcastInArgs.info.ageable = false;
        setUcastInArgs.info.trunk   = false;
        EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], gTestApp.hostMacAddr);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

        ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
        }
    }
    /* Wait for link up */
    if ((status == ENET_SOK) && (gTestApp.testLoopBackType == LOOPBACK_TYPE_PHY))
    {
        status = TestApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */
    if (status == ENET_SOK)
    {
        TestApp_createRxTxTasks();

        SemaphoreP_pend(&gTestApp.txDoneSemObj, SystemP_WAIT_FOREVER);
        SemaphoreP_pend(&gTestApp.rxDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Disable CPTS event */
    EnetMacPort_GenericInArgs inArgs;
    inArgs.macPort = gTestApp.macPort;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
	ENET_IOCTL(gTestApp.hEnet,
			   gTestApp.coreId,
			   CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT,
			   &prms,
			   status);
	EnetAppUtils_assert(status == ENET_SOK);

    /* Print network statistics */
    if (status == ENET_SOK)
    {
        if (Enet_isCpswFamily(gTestApp.enetType))
        {
            TestApp_showCpswStats();
        }
    }

    /* Print DMA statistics */
    if (status == ENET_SOK)
    {
        EnetAppUtils_showRxChStats(gTestApp.hRxCh);
        EnetAppUtils_showTxChStats(gTestApp.hTxCh);
    }

    /* Close Enet DMA driver */
    TestApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gTestApp.enetType, gTestApp.instId, gTestApp.coreId, gTestApp.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gTestApp.enetType, gTestApp.instId);
    gTestApp.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gTestApp.enetType, gTestApp.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gTestApp.rxSemObj);
    SemaphoreP_destruct(&gTestApp.rxDoneSemObj);
    SemaphoreP_destruct(&gTestApp.txSemObj);
    SemaphoreP_destruct(&gTestApp.txDoneSemObj);

    if((gTestApp.testResult == LOOPBACK_TEST_PASS) && (status == ENET_SOK))
    {
        EnetAppUtils_print("CptsRxTimeStamping:MCUSDK-11587:PASS\r\n");
    }
    else
    {
        EnetAppUtils_print("CptsRxTimeStamping:MCUSDK-11587:FAIL\r\n");
    }

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void TestApp_createRxTxTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 3U;
    taskParams.stack          = gTestAppTaskStackRx;
    taskParams.stackSize      = sizeof(gTestAppTaskStackRx);
    taskParams.args           = (void*)&gTestApp.rxSemObj;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &TestApp_rxTask;

    status = TaskP_construct(&gTestApp.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gTestAppTaskStackTx;
    taskParams.stackSize      = sizeof(gTestAppTaskStackTx);
    taskParams.args           = (void*)&gTestApp.txSemObj;
    taskParams.name           = "Tx Task";
    taskParams.taskMain       = &TestApp_txTask;

    status = TaskP_construct(&gTestApp.txTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static uint32_t TestApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gTestApp.hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pktInfo->node);
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

static void TimeSync_addHalfWord(volatile uint8_t *src,
                          uint16_t halfWord)
{
    *(src) = (halfWord >> 8) & 0xff;
    *(src + 1) = (halfWord) & 0xff;
}

static void TestApp_txTask(void *args)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t txRetrievePktCnt;
    uint32_t loopCnt, pktCnt;
    int32_t status = ENET_SOK;
    uint8_t gptpMcastAddr[ENET_MAC_ADDR_LEN] = {0x01U, 0x80U, 0xC2U, 0x00U, 0x00U, 0x0EU};

    gTestApp.totalTxCnt = 0U;
    gTestApp.testResult = LOOPBACK_TEST_PASS;
    for (loopCnt = 0U; loopCnt < ENETLPBK_NUM_ITERATION; loopCnt++)
    {
        pktCnt = 0U;
        while (pktCnt < ENETLPBK_TEST_PKT_NUM)
        {
            /* Transmit a single packet */
            EnetQueue_initQ(&txSubmitQ);

            /* Dequeue one free TX Eth packet */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.txFreePktInfoQ);

            while (NULL != pktInfo)
            {
                pktCnt++;
                /* Fill the TX Eth frame with test content */
                frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                memcpy(frame->hdr.dstMac, gptpMcastAddr, ENET_MAC_ADDR_LEN);
                memcpy(frame->hdr.srcMac, &gTestApp.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
                frame->hdr.etherType = Enet_htons(ETHERTYPE_PTP_V2_FRAME_TYPE);
                /* Fill the packet and seqId */
                memcpy(frame->payload, &gptpPktPayload[0U], 46U);
                TimeSync_addHalfWord((uint8_t *)frame + 44U, pktCnt);

                pktInfo->sgList.list[0].segmentFilledLen = 46U + sizeof(EthFrameHeader);
                pktInfo->sgList.numScatterSegments = 1;
                pktInfo->chkSumInfo = 0U;
                pktInfo->appPriv    = &gTestApp;
                EnetDma_checkPktState(&pktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_FREEQ,
                                      ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Enqueue the packet for later transmission */
                EnetQueue_enq(&txSubmitQ, &pktInfo->node);

                if (pktCnt >= ENETLPBK_TEST_PKT_NUM)
                {
                    break;
                }

                /* Dequeue one free TX Eth packet */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.txFreePktInfoQ);
            }

            while (0U != EnetQueue_getQCount(&txSubmitQ))
            {
                uint32_t txCnt = EnetQueue_getQCount(&txSubmitQ);
                status = EnetDma_submitTxPktQ(gTestApp.hTxCh,
                                              &txSubmitQ);
                SemaphoreP_pend(&gTestApp.txSemObj, SystemP_WAIT_FOREVER);

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
                        txRetrievePktCnt += TestApp_retrieveFreeTxPkts();
                    }
                }
                else
                {
                    break;
                }
            }
        }

        gTestApp.totalTxCnt += pktCnt;
    }

    EnetAppUtils_print("Transmitted %d packets \r\n", gTestApp.totalTxCnt);
    SemaphoreP_post(&gTestApp.txDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_txTask() and exit..\r\n");
    TaskP_destruct(&gTestApp.txTaskObj);
    TaskP_exit();
}

static uint32_t TestApp_receivePkts(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gTestApp.hRxCh, &rxReadyQ);
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

            EnetQueue_enq(&gTestApp.rxReadyQ, &pktInfo->node);
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

static int32_t TestApp_getRxTimestamp(uint16_t seqId, uint32_t *nanoseconds,
        uint64_t *seconds)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint64_t tsVal;
    static uint32_t count = 0;

    inArgs.msgType = 0U; //Sync
    inArgs.seqId = seqId;
    inArgs.portNum = 0U;
    inArgs.domain = 0U;
    if (seqId != ((count % ENETLPBK_TEST_PKT_NUM) + 1))
    {
        gTestApp.testResult = LOOPBACK_TEST_FAIL;
    }
    count++;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
    ENET_IOCTL(gTestApp.hEnet, gTestApp.coreId,
            ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP, &prms, status);
    if (status != ENET_SOK)
    {
        tsVal = 0U;
        gTestApp.testResult = LOOPBACK_TEST_FAIL;
    }
    *nanoseconds = (uint32_t) (tsVal % (uint64_t) TIME_SEC_TO_NS);
    *seconds = tsVal / (uint64_t) TIME_SEC_TO_NS;

    return status;
}

void TimeSync_convEndianess(volatile void *src,
                            volatile void *dst,
                            uint8_t numBytes)
{
    uint8_t i;
    uint8_t *srcPtr = (uint8_t *)src;
    uint8_t *dstPtr = (uint8_t *)dst;

    /*If multiple of 2*/
    if ((numBytes & 0x1) == 0)
    {
        dstPtr = dstPtr + numBytes - 1;

        for (i = 0; i < numBytes; i++)
        {
            *(dstPtr--) = *(srcPtr++);
        }
    }
}

static void TestApp_rxTask(void *args)
{
    EnetDma_Pkt *pktInfo;
    uint32_t rxReadyCnt;
    uint32_t loopCnt, loopRxPktCnt;
    int32_t status = ENET_SOK;
    uint16_t seqId = 0U;
    uint32_t nanoseconds;
    uint64_t seconds;

    gTestApp.totalRxCnt = 0U;

    for (loopCnt = 0U; loopCnt < ENETLPBK_NUM_ITERATION; loopCnt++)
    {
        loopRxPktCnt = 0U;
        /* Wait for packet reception */
        do
        {
            SemaphoreP_pend(&gTestApp.rxSemObj, SystemP_WAIT_FOREVER);
            /* Get the packets received so far */
            rxReadyCnt = TestApp_receivePkts();
            if (rxReadyCnt > 0U)
            {
                /* Consume the received packets and release them */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.rxReadyQ);
                while (NULL != pktInfo)
                {
                    EnetDma_checkPktState(&pktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_READYQ,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);
                    /* Get the seqId and retrieve the corresponding time stamp */
                    TimeSync_convEndianess(pktInfo->sgList.list[0].bufPtr + 44U, &seqId, 2);
                    TestApp_getRxTimestamp(seqId, &nanoseconds, &seconds);
                    /* Release the received packet */
                    EnetQueue_enq(&gTestApp.rxFreeQ, &pktInfo->node);
                    pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.rxReadyQ);
                }

                /*Submit now processed buffers */
                if (status == ENET_SOK)
                {
                    EnetAppUtils_validatePacketState(&gTestApp.rxFreeQ,
                                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                                     ENET_PKTSTATE_APP_WITH_DRIVER);

                    EnetDma_submitRxPktQ(gTestApp.hRxCh,
                                         &gTestApp.rxFreeQ);
                }
            }

            loopRxPktCnt += rxReadyCnt;
        }
        while (loopRxPktCnt < ENETLPBK_TEST_PKT_NUM);

        gTestApp.totalRxCnt += loopRxPktCnt;
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", ENETLPBK_TEST_PKT_NUM, gTestApp.totalRxCnt);
    }
    else
    {
        EnetAppUtils_print("Received %d packets\r\n", gTestApp.totalRxCnt);
    }

    SemaphoreP_post(&gTestApp.rxDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_rxTask() and exit..\r\n");
    TaskP_destruct(&gTestApp.rxTaskObj);
    TaskP_exit();
}

static void TestApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gTestApp.rxSemObj);
}

static void TestApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(&gTestApp.txSemObj);
}

static void TestApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    /* Initialize all queues */
    EnetQueue_initQ(&gTestApp.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < gTestCfg.txNumPkt ; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

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
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    EnetQueue_initQ(&gTestApp.rxFreeQ);
    EnetQueue_initQ(&gTestApp.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < gTestCfg.txNumPkt; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                        ENETDMA_CACHELINE_ALIGNMENT,
                                        ENET_ARRAYSIZE(scatterSegments),
                                        scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gTestApp.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gTestApp.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gTestApp.hRxCh,
                         &gTestApp.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gTestApp.rxFreeQ));
}

static int32_t TestApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo;

        txInArgs.cbArg   = &gTestApp;
        txInArgs.notifyCb = TestApp_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gTestApp.hTxCh   = txChInfo.hTxCh;


        TestApp_initTxFreePktQ();

        if (NULL != gTestApp.hTxCh)
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

        rxInArgs.notifyCb = TestApp_rxIsrFxn;
        rxInArgs.cbArg   = &gTestApp;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gTestApp.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gTestApp.hostMacAddr, &rxChInfo.macAddr[rxChInfo.numValidMacAddress-1][0]);
        if (NULL == gTestApp.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gTestApp.hRxCh);
        }
        else
        {
            /* Submit all ready RX buffers to DMA.*/
            TestApp_initRxReadyPktQ();
        }
    }

    return status;
}

static void TestApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gTestApp.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gTestApp.hEnet,
                       gTestApp.coreKey,
                       gTestApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gTestApp.hEnet,
                       gTestApp.coreKey,
                       gTestApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gTestApp.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gTestApp.txFreePktInfoQ);

}
