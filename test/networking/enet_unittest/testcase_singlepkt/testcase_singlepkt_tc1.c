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
 * \file  testcase_singlepkt_tc1.c
 *
 * \brief This file contains the Testcase 1 implementation where there is only 
 *        task for both TX and RX.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "FreeRTOS.h"
#include "testcase_common_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */
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

static void TestApp_rxIsrFxn(void *appData);

static void TestApp_sendReceivePkts();

static void TestApp_initTxFreePkt(uint32_t txScatterSeg);

static void TestApp_initRxReadyPkt(uint32_t rxScatterSeg);

static int32_t TestApp_openDma(void);

static void TestApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern TestApp_Obj gTestApp;
extern TestCfg_Obj gTestCfg;

static uint32_t  ScatterSegment1[] = {ENET_MEM_LARGE_POOL_PKT_SIZE};
static uint32_t  ScatterSegment2[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/2, ENET_MEM_LARGE_POOL_PKT_SIZE/2};
static uint32_t  ScatterSegment3[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/3, ENET_MEM_LARGE_POOL_PKT_SIZE/3, (ENET_MEM_LARGE_POOL_PKT_SIZE/3 + ENET_MEM_LARGE_POOL_PKT_SIZE%3)};
static uint32_t  ScatterSegment4[] = {ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4, ENET_MEM_LARGE_POOL_PKT_SIZE/4};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TestApp_SinglePktTestcase(void)
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
        TestApp_sendReceivePkts();

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

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void TestApp_sendReceivePkts()
{
    EthFrame *frame;
    uint32_t pktCnt;
    int32_t status = ENET_SOK;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};

    gTestApp.totalRxCnt = 0U;

    gTestApp.totalTxCnt = 0U;

    pktCnt = 0U;
    while (pktCnt < ENETLPBK_TEST_PKT_NUM)
    {
        EnetAppUtils_assert(gTestApp.txFreePkt != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&gTestApp.txFreePkt->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        if (NULL != gTestApp.txFreePkt)
        {
            pktCnt++;
            /* Fill the TX Eth frame with test content */
            frame = (EthFrame *)gTestApp.txFreePkt->sgList.list[0].bufPtr;
            memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
            memcpy(frame->hdr.srcMac, &gTestApp.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
            frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
            memset(&frame->payload[0U], (uint8_t)(0xA5 + pktCnt), ENETLPBK_TEST_PKT_LEN);

            gTestApp.txFreePkt->sgList.list[0].segmentFilledLen = ENETLPBK_TEST_PKT_LEN + sizeof(EthFrameHeader);
            gTestApp.txFreePkt->sgList.numScatterSegments = 1;
            gTestApp.txFreePkt->chkSumInfo = 0U;
            gTestApp.txFreePkt->appPriv    = &gTestApp;
            EnetDma_checkPktState(&gTestApp.txFreePkt->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Submit the packet for transmission */
            status = EnetDma_submitTxPkt(gTestApp.hTxCh, gTestApp.txFreePkt);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("EnetDma_submitTxPkt() failed to submit pkts: %d\r\n", status);
            }

            /* Setting the pkt to NULL for retrieval of another packet*/
            gTestApp.txFreePkt = NULL;

            /* Waiting for RX ISR*/
            SemaphoreP_pend(&gTestApp.rxSemObj, SystemP_WAIT_FOREVER);

            /* Get the packet received so far */
            status = EnetDma_retrieveRxPkt(gTestApp.hRxCh, &gTestApp.rxReadyPkt);

            /* Consume the received packets and release them */
            if (NULL != gTestApp.rxReadyPkt)
            {
                gTestApp.totalRxCnt++;
                EnetDma_checkPktState(&gTestApp.rxReadyPkt->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_READYQ,
                                      ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Consume the packet by just printing its content */
                if (gTestApp.printFrame)
                {
                    uint32_t packetPrintLen;

                    frame = (EthFrame *)gTestApp.rxReadyPkt->sgList.list[0].bufPtr;
                    packetPrintLen = gTestApp.rxReadyPkt->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);

                    EnetAppUtils_printFrame(frame,
                                            packetPrintLen);
                }
                EnetAppUtils_assert(EnetLpbk_verifyRxFrame(gTestApp.rxReadyPkt, pktCnt) == true);
                /* Release the received packet */
                EnetDma_checkPktState(&gTestApp.rxReadyPkt->pktState,
                            ENET_PKTSTATE_MODULE_APP,
                            ENET_PKTSTATE_APP_WITH_FREEQ,
                            ENET_PKTSTATE_APP_WITH_DRIVER);
                EnetDma_submitRxPkt(gTestApp.hRxCh, gTestApp.rxReadyPkt);

                /* Setting the rx Pkt to NULL for next retreival*/
                gTestApp.rxReadyPkt = NULL;
            }

            status = EnetDma_retrieveTxPkt(gTestApp.hTxCh, &gTestApp.txFreePkt);
            if (status == ENET_SOK)
            {

                if (NULL != gTestApp.txFreePkt)
                {
                    EnetDma_checkPktState(&gTestApp.txFreePkt->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_DRIVER,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);
                }
            }
        }
    }
    gTestApp.totalTxCnt += pktCnt;

    EnetAppUtils_print("Transmitted %d packets \r\n", gTestApp.totalTxCnt);
    EnetAppUtils_print("Recieved %d packets \r\n", gTestApp.totalRxCnt);
    SemaphoreP_post(&gTestApp.rxDoneSemObj);

    EnetAppUtils_print("Delete TestApp_txTask() and exit..\r\n");
}

static void TestApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gTestApp.rxSemObj);
}

static void TestApp_initTxFreePkt(uint32_t txScatterSeg)
{
    EnetAppUtils_assert(txScatterSeg >= 1 && txScatterSeg <= 4);

    switch (txScatterSeg)
    {
        case 1U:
            gTestApp.txFreePkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment1),
                                                           ScatterSegment1);
            break;
        case 2U:
            gTestApp.txFreePkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment2),
                                                           ScatterSegment2);
            break;
        case 3U:
            gTestApp.txFreePkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment3),
                                                           ScatterSegment3);
            break;
        case 4U:
            gTestApp.txFreePkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment4),
                                                           ScatterSegment4);
            break;
        default: break;
    }

    EnetAppUtils_assert(gTestApp.txFreePkt != NULL);
    ENET_UTILS_SET_PKT_APP_STATE(&gTestApp.txFreePkt->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
}

static void TestApp_initRxReadyPkt(uint32_t rxScatterSeg)
{
    EnetAppUtils_assert(rxScatterSeg >= 1 && rxScatterSeg <= 4);

    switch (rxScatterSeg)
    {
        case 1U:
            gTestApp.rxReadyPkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment1),
                                                           ScatterSegment1);
            break;
        case 2U:
            gTestApp.rxReadyPkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment2),
                                                           ScatterSegment2);
            break;
        case 3U:
            gTestApp.rxReadyPkt = EnetMem_allocEthPkt(&gTestApp,
                                                           ENETDMA_CACHELINE_ALIGNMENT,
                                                           ENET_ARRAYSIZE(ScatterSegment3),
                                                           ScatterSegment3);
            break;
        case 4U:
            gTestApp.rxReadyPkt = EnetMem_allocEthPkt(&gTestApp,
                                                      ENETDMA_CACHELINE_ALIGNMENT,
                                                      ENET_ARRAYSIZE(ScatterSegment4),
                                                      ScatterSegment4);
            break;
        default: break;
    }

    EnetAppUtils_assert(gTestApp.rxReadyPkt != NULL);
    ENET_UTILS_SET_PKT_APP_STATE(&gTestApp.rxReadyPkt->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

    EnetDma_checkPktState(&gTestApp.rxReadyPkt->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_FREEQ,
                                ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPkt(gTestApp.hRxCh, gTestApp.rxReadyPkt);
}

static int32_t TestApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo;

        txInArgs.cbArg    = NULL;
        txInArgs.notifyCb = NULL;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gTestApp.hTxCh   = txChInfo.hTxCh;

        TestApp_initTxFreePkt(gTestCfg.txScatterSeg);

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
            TestApp_initRxReadyPkt(gTestCfg.rxScatterSeg);
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

    EnetMem_freeEthPkt(gTestApp.txFreePkt);
    //EnetMem_freeEthPkt(gTestApp.rxReadyPkt);
    //EnetAppUtils_freePktInfoQ(&gTestApp.rxFreeQ);
    //EnetAppUtils_freePktInfoQ(&gTestApp.txFreePktInfoQ);

}
