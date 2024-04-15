/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include "dataflow.h"
#include "enetapp_icssg.h"

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
static uint8_t gEnetAppTaskStackRx[ENET_SYSCFG_MAX_ENET_INSTANCES][ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

static void EnetApp_initTxFreePktQ(void);
static uint32_t EnetApp_retrieveFreeTxPkts(EnetApp_PerCtxt *perCtxt);
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

/* Rx Isr for non-gPTP traffic */
static void EnetApp_rxIsrFxn(void *appData)
{
    EnetApp_PerCtxt *perCtxt = (EnetApp_PerCtxt *)appData;
    SemaphoreP_post(&perCtxt->rxSemObj);
}

/* Open non-gPTP channels */
static int32_t EnetApp_openDma(EnetApp_PerCtxt *perCtxt)
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    for (uint32_t chNum = 0U; chNum < perCtxt->nonPtpTxChNum; chNum++)
    {
        txInArgs.cbArg   = NULL;
        txInArgs.notifyCb = NULL;
        EnetApp_getTxDmaHandle(perCtxt->nonPtpTxChId[chNum],
                               &txInArgs,
                               &txChInfo);
        perCtxt->txChNum[chNum] = txChInfo.txChNum;
        perCtxt->hTxCh[chNum]   = txChInfo.hTxCh;
        if (perCtxt->hTxCh[chNum] == NULL)
        {
#if FIX_RM
            /* Free the channel number if open Tx channel failed */
            EnetAppUtils_freeTxCh(perCtxt->hEnet,
                                  perCtxt->coreKey,
                                  gEnetAppCfg.coreId,
                                  perCtxt->txChNum[chNum]);
#endif
            EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(perCtxt->hTxCh[chNum] != NULL);
         }
        /* Allocate TX packets and keep them locally enqueued */
        if (status == ENET_SOK)
        {
            EnetApp_initTxFreePktQ();
        }
    }
    /* Open the RX flow */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = perCtxt;
        for(uint32_t flowNum = 0U; flowNum < perCtxt->nonPtpRxFlowNum; flowNum++)
        {
            EnetApp_getRxDmaHandle(perCtxt->nonPtpRxFlowId[flowNum],
                                   &rxInArgs,
                                   &rxChInfo);
            perCtxt->rxStartFlowIdx[flowNum] = rxChInfo.rxFlowStartIdx;
            perCtxt->rxFlowIdx[flowNum] = rxChInfo.rxFlowIdx;
            perCtxt->hRxCh[flowNum]  = rxChInfo.hRxCh;
            if(flowNum == 0U)
            {
                EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1U);
                EnetUtils_copyMacAddr(perCtxt->macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
                EnetAppUtils_print("MAC port addr: ");
                EnetAppUtils_printMacAddr(perCtxt->macAddr);
            }
            if (perCtxt->hRxCh[flowNum] == NULL)
            {
                EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
                status = ENET_EFAIL;
                EnetAppUtils_assert(perCtxt->hRxCh[flowNum] != NULL);
            }
            /* Submit all ready RX buffers to DMA */
            EnetApp_initRxReadyPktQ(perCtxt->hRxCh[flowNum]);
        }
    }
    return status;
}

static void EnetApp_closeDma(EnetApp_PerCtxt *perCtxt)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    for(uint32_t i = 0U; i < perCtxt->nonPtpRxFlowNum; i++)
    {
       /* Close Regular RX channel */
       EnetApp_closeRxDma(perCtxt->nonPtpRxFlowId[i],
                          perCtxt->hEnet,
                          perCtxt->coreKey,
                          gEnetAppCfg.coreId,
                          &fqPktInfoQ,
                          &cqPktInfoQ);

       EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
       EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts(perCtxt);

    for (uint32_t chIdx = 0; chIdx < perCtxt->nonPtpTxChNum; chIdx++)
    {
        EnetApp_closeTxDma(perCtxt->nonPtpTxChId[chIdx],
                           perCtxt->hEnet,
                           perCtxt->coreKey,
                           gEnetAppCfg.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetAppCfg.txFreePktInfoQ);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_TX_PKT/ENET_SYSCFG_TX_CHANNELS_NUM); i++)
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

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i= 0; i< ENET_SYSCFG_TOTAL_NUM_RX_PKT/ENET_SYSCFG_RX_FLOWS_NUM; i++)
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

static uint32_t EnetApp_retrieveFreeTxPkts(EnetApp_PerCtxt *perCtxt)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t i, txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    for (i = 0; i < perCtxt->nonPtpTxChNum; i++)
    {
        status = EnetDma_retrieveTxPktQ(perCtxt->hTxCh[i], &txFreeQ);
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
    }

    return txFreeQCnt;
}

/* Rx Echo task for non-gPTP traffic */
static void EnetApp_rxTask(void *args)
{
    EnetApp_PerCtxt *perCtxt = (EnetApp_PerCtxt *)args;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    EthFrame *txFrame;
    uint32_t flowIdx;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s:%d: default RX flow started\r\n",
            perCtxt->name, perCtxt->perIdx);

    while (ENET_SOK == status)
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&perCtxt->rxSemObj, SystemP_WAIT_FOREVER);
        for (flowIdx = 0; flowIdx < perCtxt->nonPtpRxFlowNum; flowIdx++)
        {
            EnetQueue_initQ(&rxReadyQ);
            EnetQueue_initQ(&rxFreeQ);
            EnetQueue_initQ(&txSubmitQ);

            /* Get the packets received so far */
            status = EnetDma_retrieveRxPktQ(perCtxt->hRxCh[flowIdx], &rxReadyQ);
            if (status != ENET_SOK)
            {
                /* Should we bail out here? */
                EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n",
                        status);
                continue;
            }
#if DEBUG
        EnetAppUtils_print("%s: Received %u packets\r\n", perCtxt->name, EnetQueue_getQCount(&rxReadyQ));
#endif

            /* Consume the received packets and send them back */
            rxPktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
            while (rxPktInfo != NULL)
            {
                rxFrame = (EthFrame*) rxPktInfo->sgList.list[0].bufPtr;
                EnetDma_checkPktState(&rxPktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_DRIVER,
                        ENET_PKTSTATE_APP_WITH_READYQ);

                /* Retrieve TX packets from driver and recycle them */
                EnetApp_retrieveFreeTxPkts(perCtxt);

                /* Dequeue one free TX Eth packet */
                txPktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);
                if (txPktInfo != NULL)
                {
                    /* Fill the TX Eth frame with test content */
                    txFrame = (EthFrame*) txPktInfo->sgList.list[0].bufPtr;
                    memcpy(txFrame->hdr.dstMac, rxFrame->hdr.srcMac,
                            ENET_MAC_ADDR_LEN);
                    memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U],
                            ENET_MAC_ADDR_LEN);
                    txFrame->hdr.etherType = rxFrame->hdr.etherType;

                    txPktInfo->sgList.list[0].segmentFilledLen =
                            rxPktInfo->sgList.list[0].segmentFilledLen;
                    EnetAppUtils_assert(
                            txPktInfo->sgList.list[0].segmentAllocLen
                                    >= txPktInfo->sgList.list[0].segmentFilledLen);
                    memcpy(&txFrame->payload[0U], &rxFrame->payload[0U],
                            rxPktInfo->sgList.list[0].segmentFilledLen
                                    - sizeof(EthFrameHeader));

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
                    EnetAppUtils_print(
                            "%s: Drop due to TX pkt not available\r\n",
                            perCtxt->name);
                }

                EnetDma_checkPktState(&rxPktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP, ENET_PKTSTATE_APP_WITH_READYQ,
                        ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Release the received packet */
                EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
                rxPktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
            }

            /* Transmit all enqueued packets */
            status = EnetDma_submitTxPktQ(perCtxt->hTxCh[0], &txSubmitQ);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to submit TX pkt queue: %d\r\n",
                        perCtxt->name, status);
            }

            EnetAppUtils_validatePacketState(&rxFreeQ,
                    ENET_PKTSTATE_APP_WITH_FREEQ,
                    ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Submit now processed buffers */
            EnetDma_submitRxPktQ(perCtxt->hRxCh[flowIdx], &rxFreeQ);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to submit RX pkt queue: %d\r\n",
                                    perCtxt->name, status);
            }
        }
    }
    TaskP_exit();
}

void EnetApp_createRxTask(EnetApp_PerCtxt *perCtxt)
{
    TaskP_Params taskParams;
    int32_t status;

    status = EnetApp_openDma(perCtxt);
    DebugP_assert(ENET_SOK == status);

    status = SemaphoreP_constructBinary(&perCtxt->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = &gEnetAppTaskStackRx[perCtxt->perIdx][0];
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx[perCtxt->perIdx]);
    taskParams.args           = (void*)perCtxt;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&perCtxt->rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask(EnetApp_PerCtxt *perCtxt)
{
    SemaphoreP_destruct(&perCtxt->rxSemObj);
    TaskP_destruct(&perCtxt->rxTaskObj);
    EnetApp_closeDma(perCtxt);
}
