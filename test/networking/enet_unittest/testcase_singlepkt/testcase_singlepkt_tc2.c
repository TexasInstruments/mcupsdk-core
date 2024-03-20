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
 * \file  testcase_singlepkt_tc2.c
 *
 * \brief This file contains the loopback test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "FreeRTOS.h"
#include "testcase_common_cfg.h"
#include "testcase_singlepkt_tc2.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


 volatile uint32_t gProfilerArr[PROFILE_TIME_MAX__TIME_ENTRIES][PROFILE_TIME_MAX_COLUMN_ENTRIES];
uint8_t gProfilerIdx;
volatile uint32_t gTimeTaken[40];
#define PROFILE_TIME(time) ({\
                            gProfilerArr[gProfilerIdx][0U] = time;\
                            gProfilerArr[gProfilerIdx++][1U] = __LINE__;\
                            })
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void TestApp_createRxTxTasks(void);

static void TestApp_txTask();

static void TestApp_rxTask(void *args);

static void TestApp_rxIsrFxn(void *appData);

static void TestApp_txIsrFxn(void *appData);

static void TestApp_initTxFreePktQ(uint32_t txNumPkt, uint32_t txScatterSeg);

static void TestApp_initRxReadyPktQ(uint32_t rxNumPkt, uint32_t rxScatterSeg);

static int32_t TestApp_openDma(void);

static void TestApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gTestAppTaskStackTx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gTestAppTaskStackRx[ENETLPBK_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern TestApp_Obj gTestApp;
extern TestCfg_Obj gTestCfg;
static uint32_t  ScatterSegment1[] = {ENET_MEM_LARGE_POOL_PKT_SIZE};
static uint32_t  ScatterSegment2[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/2, ENET_MEM_LARGE_POOL_PKT_SIZE/2};
static uint32_t  ScatterSegment3[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/3, ENET_MEM_LARGE_POOL_PKT_SIZE/3, (ENET_MEM_LARGE_POOL_PKT_SIZE/3 + ENET_MEM_LARGE_POOL_PKT_SIZE%3)};
static uint32_t  ScatterSegment4[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TestApp_SinglePktTestcase2(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;
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

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

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

static void TestApp_txTask()
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t pktCnt;
    uint32_t txSubmitPktCnt, txRetrievePktCnt;
    int32_t status = ENET_SOK;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};

    gTestApp.totalTxCnt = 0U;

    pktCnt = 0U;
    txSubmitPktCnt = 0U;
    txRetrievePktCnt = 0U;
    while (pktCnt < ENETLPBK_TEST_PKT_NUM)
    {
        /* Dequeue one free TX Eth packet */
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.txFreePktInfoQ);

        while (NULL != pktInfo)
        {
            pktCnt++;

            /* Fill the TX Eth frame with test content */
            frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
            memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
            memcpy(frame->hdr.srcMac, &gTestApp.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
            frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
            memset(&frame->payload[0U], (uint8_t)(0xA5 + pktCnt), ENETLPBK_TEST_PKT_LEN);

            pktInfo->sgList.list[0].segmentFilledLen = ENETLPBK_TEST_PKT_LEN + sizeof(EthFrameHeader);
            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;
            pktInfo->appPriv    = &gTestApp;
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Submit the single packet for transmission */
            status = EnetDma_submitTxPkt(gTestApp.hTxCh, pktInfo);
            ClockP_usleep(1000);
            txSubmitPktCnt++;

            if (status != ENET_SOK)
            {
                EnetAppUtils_print("EnetDma_submitTxPkt() failed to submit pkts: %d\r\n",
                               status);
            }

            if (pktCnt >= ENETLPBK_TEST_PKT_NUM)
            {
                break;
            }

            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gTestApp.txFreePktInfoQ);
        }

        SemaphoreP_pend(&gTestApp.txSemObj, SystemP_WAIT_FOREVER);

        while(txSubmitPktCnt != txRetrievePktCnt)
        {
            status = EnetDma_retrieveTxPkt(gTestApp.hTxCh, &pktInfo);
            if (status == ENET_SOK)
            {
                if (NULL != pktInfo)
                {
                    txRetrievePktCnt++;
                    EnetDma_checkPktState(&gTestApp.txFreePkt->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_DRIVER,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);

                    EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pktInfo->node);
                }
            }
        }
    }

    gTestApp.totalTxCnt += pktCnt;

    EnetAppUtils_print("Transmitted %d packets \r\n", gTestApp.totalTxCnt);
    SemaphoreP_post(&gTestApp.txDoneSemObj);

    EnetAppUtils_print("Delete TestApp_txTask() and exit..\r\n");
    TaskP_destruct(&gTestApp.txTaskObj);
    TaskP_exit();
}

static void TestApp_rxTask(void *args)
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    int32_t status = ENET_SOK;
    uint32_t rxPktCnt;

    gTestApp.totalRxCnt = 0U;
    rxPktCnt     = 0U;

    do
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gTestApp.rxSemObj, SystemP_WAIT_FOREVER);

        /* Get the packet received so far */
        status = EnetDma_retrieveRxPkt(gTestApp.hRxCh, &pktInfo);

        if (NULL != pktInfo)
        {
            rxPktCnt++;
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_READYQ,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Consume the packet by just printing its content */
            if (gTestApp.printFrame)
            {
                uint32_t packetPrintLen;

                frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                packetPrintLen = pktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);

                EnetAppUtils_printFrame(frame,
                                        packetPrintLen);
            }
            EnetAppUtils_assert(EnetLpbk_verifyRxFrame(pktInfo, rxPktCnt) == true);

            /* Release the received packet */
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);
            EnetDma_submitRxPkt(gTestApp.hRxCh, pktInfo);
        }
    } while (rxPktCnt < ENETLPBK_TEST_PKT_NUM);

    gTestApp.totalRxCnt += rxPktCnt;

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", ENETLPBK_TEST_PKT_NUM, gTestApp.totalRxCnt);
    }
    else
    {
        EnetAppUtils_print("Received %d packets\r\n", gTestApp.totalRxCnt);
    }

    SemaphoreP_post(&gTestApp.rxDoneSemObj);

    EnetAppUtils_print("Delete TestApp_rxTask() and exit..\r\n");
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

static void TestApp_initTxFreePktQ(uint32_t txNumPkt, uint32_t txScatterSeg)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    EnetAppUtils_assert(txScatterSeg >= 1 && txScatterSeg <= 4);

    /* Initialize all queues */
    EnetQueue_initQ(&gTestApp.txFreePktInfoQ);

    switch (txScatterSeg)
    {
        case 1U:
            for (i = 0U; i < txNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment1),
                                               ScatterSegment1);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pPktInfo->node);
            }
            break;
        case 2U:
            for (i = 0U; i < txNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment2),
                                               ScatterSegment2);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pPktInfo->node);
            }
            break;
        case 3U:
            for (i = 0U; i < txNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment3),
                                               ScatterSegment3);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pPktInfo->node);
            }
            break;
        case 4U:
            for (i = 0U; i < txNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment4),
                                               ScatterSegment4);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.txFreePktInfoQ, &pPktInfo->node);
            }
            break;
        default: break;
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                        EnetQueue_getQCount(&gTestApp.txFreePktInfoQ));
}

static void TestApp_initRxReadyPktQ(uint32_t rxNumPkt, uint32_t rxScatterSeg)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;

    EnetQueue_initQ(&gTestApp.rxFreeQ);
    EnetQueue_initQ(&rxReadyQ);

    switch (rxScatterSeg)
    {
        case 1U:
            for (i = 0U; i < rxNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment1),
                                               ScatterSegment1);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);
            }
            break;
        case 2U:
            for (i = 0U; i < rxNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment2),
                                               ScatterSegment2);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);
            }
            break;
        case 3U:
            for (i = 0U; i < rxNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment3),
                                               ScatterSegment3);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);
            }
            break;
        case 4U:
            for (i = 0U; i < rxNumPkt; i++)
            {
                pPktInfo = EnetMem_allocEthPkt(&gTestApp,
                                               ENETDMA_CACHELINE_ALIGNMENT,
                                               ENET_ARRAYSIZE(ScatterSegment4),
                                               ScatterSegment4);
                EnetAppUtils_assert(pPktInfo != NULL);
                ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&gTestApp.rxFreeQ, &pPktInfo->node);
            }
            break;
        default: break;
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gTestApp.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gTestApp.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gTestApp.hRxCh, &gTestApp.rxFreeQ);

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

        TestApp_initTxFreePktQ(gTestCfg.txNumPkt, gTestCfg.txScatterSeg);

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
            TestApp_initRxReadyPktQ(gTestCfg.txNumPkt, gTestCfg.txScatterSeg);
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
    //EnetAppUtils_assert(0U == EnetQueue_getQCount(&gTestApp.rxReadyQ));

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

    //EnetMem_freeEthPkt(gTestApp.txFreePkt);
    //EnetMem_freeEthPkt(gTestApp.rxReadyPkt);
    EnetAppUtils_freePktInfoQ(&gTestApp.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gTestApp.txFreePktInfoQ);

}
