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
 * \file  enet_cpsw_est_dataflow.c
 *
 * \brief This file contains the implementation of the APIs for data flow
 *        for the CPSW EST example app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_est_common.h"
#include "enet_cpsw_est_cfg.h"
#include "enet_cpsw_est_dataflow.h"
#include "ti_enet_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_VLAN_TPID                            (0x8100U)
#define ENETAPP_VLAN_PCP_OFFSET                      (13U)
#define ENETAPP_VLAN_PCP_MASK                        (0x7U)
#define ENETAPP_VLAN_DEI_OFFSET                      (12U)
#define ENETAPP_VLAN_DEI_MASK                        (0x1U)
#define ENETAPP_VLAN_VID_MASK                        (0xFFFU)
#define ENETAPP_VLAN_TCI(pcp, dei, vid)              ((((pcp) & ENETAPP_VLAN_PCP_MASK) << ENETAPP_VLAN_PCP_OFFSET) | \
                                                      (((dei) & ENETAPP_VLAN_DEI_MASK) << ENETAPP_VLAN_DEI_OFFSET) | \
                                                      (((vid) & ENETAPP_VLAN_VID_MASK)))

/* Experimental EtherType used in TX test packets */
#define ENETAPP_TEST_TX_ETHERTYPE                    (0x88B5U)

/* TX test packet length (total length including L2 header is 64) */
#define ENETAPP_TEST_TX_PKT_PAYLOAD_LEN              (42U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_timerCallback(ClockP_Object *clkInst, void *arg);

static void EnetApp_tickTask(void *args);

static void EnetApp_rxIsr(void *appData);

static void EnetApp_rxTask(void *args);

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

static void EnetApp_initTxFreePktQ(void);

static uint32_t EnetApp_retrieveFreeTxPkts(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetApp_rxIsr(void *appData)
{
    EnetApp_Obj *enetApp = (EnetApp_Obj *)appData;

    SemaphoreP_post(&enetApp->rxSemObj);
}

int32_t EnetApp_openDma(void)
{
    EnetApp_GetDmaHandleInArgs     rxInArgs;
    EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;


    /* Open the TX channel */
    txInArgs.cbArg    = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                           &txInArgs,
                           &txChInfo);

    gEnetApp.txChNum = txChInfo.txChNum;
    gEnetApp.hTxCh   = txChInfo.hTxCh;
    EnetAppUtils_assert(txChInfo.useGlobalEvt == true);
  
    if (gEnetApp.hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetApp.hEnet,
                              gEnetApp.coreKey,
                              gEnetApp.coreId,
                              gEnetApp.txChNum);
#endif
        EnetAppUtils_print("Failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetApp.hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    if (status == ENET_SOK)
    {
        status = EnetDma_enableTxEvent(gEnetApp.hTxCh);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        rxInArgs.notifyCb = EnetApp_rxIsr;
        rxInArgs.cbArg   = &gEnetApp;


        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);

        gEnetApp.rxStartFlowIdx = rxChInfo.rxFlowStartIdx;
        gEnetApp.rxFlowIdx = rxChInfo.rxFlowIdx;
        gEnetApp.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetApp.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);


        EnetAppUtils_assert(rxChInfo.useGlobalEvt == true);
        EnetAppUtils_assert(rxChInfo.sizeThreshEn == 0U);

        if (gEnetApp.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetApp.hRxCh != NULL);
        }
    }

    /* Allocate RX buffers and submit them to DMA */
    if (status == ENET_SOK)
    {
        EnetApp_initRxReadyPktQ(gEnetApp.hRxCh);
    }

     return status;
}

void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    /* Close RX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetApp.hEnet,
                       gEnetApp.coreKey,
                       gEnetApp.coreId,
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
                       gEnetApp.hEnet,
                       gEnetApp.coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&gEnetApp.txFreePktInfoQ);
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);
    EnetQueue_initQ(&rxReadyQ);

    /* Allocate packets from pool */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    /* Prime RX channel with free packets */
    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Allocate TX packets from pool, save them in local queue (txFreePktInfoQ) */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pPktInfo->node);
    }
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetApp.hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("Failed to retrieve TX pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

void EnetApp_createRxTask(void)
{
    TaskP_Params taskParams;
    int32_t status;

    status = SemaphoreP_constructBinary(&gEnetApp.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructCounting(&gEnetApp.rxDoneSemObj, 0, COUNTING_SEM_COUNT);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority   = 5U;
    taskParams.stack      = gEnetAppTaskStackRx;
    taskParams.stackSize  = sizeof(gEnetAppTaskStackRx);
    taskParams.args       = (void *)NULL;
    taskParams.name       = "RX Task";
    taskParams.taskMain   = &EnetApp_rxTask;

    status = TaskP_construct(&gEnetApp.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask(void)
{
    SemaphoreP_destruct(&gEnetApp.rxSemObj);
    SemaphoreP_destruct(&gEnetApp.rxDoneSemObj);
    TaskP_destruct(&gEnetApp.rxTaskObj);
}

uint64_t EnetApp_getCurrentTime(void)
{
    Enet_IoctlPrms prms;
    int32_t status;
    uint64_t tsVal = 0ULL;

    /* Software push event */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &tsVal);
    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, status);
    if (status != ENET_SOK)
    {
        tsVal = 0ULL;
    }

    return tsVal;
}

static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *rxPktInfo;
    int32_t status = ENET_SOK;

    while ((ENET_SOK == status) && gEnetApp.run)
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetApp.rxSemObj, SystemP_WAIT_FOREVER);

        /* Initialize local queues */
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);

        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(gEnetApp.hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }

        /* Consume (just drop) the received packets and send them back */
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (rxPktInfo != NULL)
        {
            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_READYQ);

            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_READYQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Release the received packet */
            EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }

        EnetAppUtils_validatePacketState(&rxFreeQ,
                                         ENET_PKTSTATE_APP_WITH_FREEQ,
                                         ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        EnetDma_submitRxPktQ(gEnetApp.hRxCh, &rxFreeQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
        }
    }

    SemaphoreP_post(&gEnetApp.rxDoneSemObj);
    TaskP_exit();
}

void EnetApp_txTest(void)
{
    EnetDma_PktQ dmaPktQ;
    EnetDma_Pkt *dmaPkt;
    EthVlanFrame *txFrame;
    uint32_t i;
    int32_t status;
    uint8_t bcastMac[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    EnetQueue_initQ(&dmaPktQ);

    /* Retrieve TX packets from driver and recycle them */
    EnetApp_retrieveFreeTxPkts();

    for (i = 0U; i < ENETAPP_TEST_TX_PKT_CNT; i++)
    {
        /* Dequeue one free TX Eth packet */
        dmaPkt = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txFreePktInfoQ);
        if (dmaPkt != NULL)
        {
            /* Fill the TX frame with test content */
            txFrame = (EthVlanFrame *)dmaPkt->sgList.list[0].bufPtr;
            memcpy(txFrame->hdr.dstMac, &bcastMac[0U], ENET_MAC_ADDR_LEN);
            memcpy(txFrame->hdr.srcMac, &gEnetApp.macAddr[0U], ENET_MAC_ADDR_LEN);

            txFrame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);
            txFrame->hdr.tci  = Enet_htons(ENETAPP_VLAN_TCI(i%8, 0, 100));
            txFrame->hdr.etherType = Enet_htons(ENETAPP_TEST_TX_ETHERTYPE);

            memset(&txFrame->payload[0U], i, ENETAPP_TEST_TX_PKT_PAYLOAD_LEN);

            dmaPkt->sgList.list[0].segmentFilledLen = sizeof(EthVlanFrameHeader) + ENETAPP_TEST_TX_PKT_PAYLOAD_LEN;
            EnetAppUtils_assert(dmaPkt->sgList.list[0].segmentAllocLen >= dmaPkt->sgList.list[0].segmentFilledLen);
            dmaPkt->appPriv         = &gEnetApp;

            EnetDma_checkPktState(&dmaPkt->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue the packet for later transmission */
            EnetQueue_enq(&dmaPktQ, &dmaPkt->node);
        }
        else
        {
            EnetAppUtils_print("Drop due to TX pkt not available\r\n");
        }
    }

    /* Submit TX packet queue to driver for transmission */
    status = EnetDma_submitTxPktQ(gEnetApp.hTxCh, &dmaPktQ);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to submit TX pkt queue: %d\r\n", status);
    }
}
