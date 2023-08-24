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
 * \file  multi_channel_dataflow.c
 *
 * \brief This file contains the implementation of the APIs for data flow for multi-channel example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "multi_channel_common.h"
#include "multi_channel_dataflow.h"
#include "ti_enet_config.h"
#include "timeSync.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENETAPP_NUM_TX_CH_PER_PERCTXT        (ENET_SYSCFG_TX_CHANNELS_NUM/gEnetApp.numPerCtxts)
#define ENETAPP_NUM_RX_FLOW_PER_PERCTXT      (ENET_SYSCFG_RX_FLOWS_NUM/gEnetApp.numPerCtxts)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t EnetApp_retrieveTxDonePkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetApp.perCtxt[0].hTxCh, &txFreeQ);
    if (status == TIMESYNC_OK)
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
        EnetAppUtils_print("EnetDma_retrieveTxPktQ() failed to retrieve pkts: %d\n",
                           status);
    }

    return txFreeQCnt;
}

void EnetApp_rxPtpNotifyFxn(void *appData)
{

    TimeSync_FrameNotifyConfig *frameNotifyCfg = NULL;
    EnetApp_PerCtxt *enet_perctxt = NULL;

    if (appData != NULL)
    {
        enet_perctxt = (EnetApp_PerCtxt *)appData;
        frameNotifyCfg = &enet_perctxt->timeSyncConfig.frameNotifyCfg;
        {
            /* Notify PTP Rx callback */
            if (frameNotifyCfg->rxNotifyCb != NULL)
            {
                frameNotifyCfg->rxNotifyCb(frameNotifyCfg->rxNotifyCbArg);
            }
        }
    }
}

void EnetApp_txPtpNotifyFxn(void *appData)
{
    uint32_t pktCount;
    EnetApp_PerCtxt *enet_perctxt = NULL;

    if (appData != NULL)
    {
        enet_perctxt = (EnetApp_PerCtxt *)appData;
        pktCount = EnetApp_retrieveFreeTxPkts(enet_perctxt->hTxPtpCh, &gEnetApp.txPtpFreePktInfoQ);
        EnetAppUtils_assert(pktCount != 0U);
    }
}

void EnetApp_cptsEvtNotifyFxn(void *pAppData, CpswCpts_Event *pEventInfo)
{
    TimeSync_TxPktInfo *pTxTsPktInfo = NULL;
    TimeSync_TxPktInfoEle *pTxTsPktInfoEle = NULL;
    TimeSync_FrameNotifyConfig *frameNotifyCfg = NULL;

    if (pAppData != NULL)
    {
        if (pEventInfo->eventType == CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT)
        {
            EnetApp_PerCtxt *enet_perctxt = (EnetApp_PerCtxt *)pAppData;
            frameNotifyCfg = &enet_perctxt->timeSyncConfig.frameNotifyCfg;

            pTxTsPktInfo = &enet_perctxt->txTsPktInfo;
            pTxTsPktInfoEle = &pTxTsPktInfo->txTsPktInfoArr[pTxTsPktInfo->rdIdx++];
            pTxTsPktInfo->rdIdx %= TIMESYNC_TX_PKTINFO_COUNT;

            /* Notify PTP Tx callback */
            if (frameNotifyCfg->txNotifyCb != NULL)
            {
                frameNotifyCfg->txNotifyCb(frameNotifyCfg->txNotifyCbArg,
                                           pTxTsPktInfoEle->portNum,
                                           pTxTsPktInfoEle->frameType,
                                           pTxTsPktInfoEle->seqId);
            }
        }
    }
}

void EnetApp_rxIsrFxn(void *appData)
{
    EnetApp_PerCtxt *perCtxt = (EnetApp_PerCtxt *)appData;

    SemaphoreP_post(&perCtxt->rxSemObj);
}

int32_t EnetApp_openDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetApp_GetDmaHandleInArgs     rxInArgs;
    EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
    EnetApp_GetDmaHandleInArgs     rxPtpInArgs;
    EnetApp_GetRxDmaHandleOutArgs  rxPtpChInfo;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    EnetApp_GetDmaHandleInArgs     txPtpInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txPtpChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg    = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                           &txInArgs,
                           &txChInfo);

    perCtxt->txChNum = txChInfo.txChNum;
    perCtxt->hTxCh   = txChInfo.hTxCh;
    EnetAppUtils_assert(txChInfo.useGlobalEvt == true);

    if (perCtxt->hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetApp.hEnet,
                              perCtxt->coreKey,
                              gEnetApp.coreId,
                              gEnetApp.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(perCtxt->hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ(&gEnetApp.txFreePktInfoQ, ENET_DMA_TX_CH0);
    }

    /* Open the TX channel for PTP Traffic */
    txPtpInArgs.cbArg = perCtxt;
    txPtpInArgs.notifyCb = EnetApp_txPtpNotifyFxn;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH_PTP + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                           &txPtpInArgs,
                           &txPtpChInfo);

    perCtxt->txPtpChNum = txPtpChInfo.txChNum;
    perCtxt->hTxPtpCh   = txPtpChInfo.hTxCh;
    EnetAppUtils_assert(txPtpChInfo.useGlobalEvt == true);

    if (perCtxt->hTxPtpCh == NULL)
    {
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(perCtxt->hTxPtpCh != NULL);
    }
    else
    {
       status = EnetDma_enableTxEvent(perCtxt->hTxPtpCh);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ(&gEnetApp.txPtpFreePktInfoQ, ENET_DMA_TX_CH_PTP);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = perCtxt;

        EnetApp_getRxDmaHandle((ENET_DMA_RX_CH0  + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT)),
                               &rxInArgs,
                               &rxChInfo);

        perCtxt->rxStartFlowIdx = rxChInfo.rxFlowStartIdx;
        perCtxt->rxFlowIdx = rxChInfo.rxFlowIdx;
        perCtxt->hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(perCtxt->macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);

        EnetAppUtils_assert(rxChInfo.useGlobalEvt == true);
        EnetAppUtils_assert(rxChInfo.sizeThreshEn == 0U);

        if (perCtxt->hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(perCtxt->hRxCh != NULL);
        }

        /*Opening another RX flow for PTP frames*/
        rxPtpInArgs.notifyCb = EnetApp_rxPtpNotifyFxn;
        rxPtpInArgs.cbArg   = perCtxt;

        EnetApp_getRxDmaHandle((ENET_DMA_RX_CH_PTP  + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT)),
                               &rxPtpInArgs,
                               &rxPtpChInfo);

        perCtxt->rxStartFlowIdx = rxPtpChInfo.rxFlowStartIdx;
        perCtxt->rxPtpFlowIdx = rxPtpChInfo.rxFlowIdx;
        perCtxt->hRxPtpCh  = rxPtpChInfo.hRxCh;
        EnetAppUtils_assert(rxPtpChInfo.numValidMacAddress == 0);

        if (perCtxt->hRxPtpCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(perCtxt->hRxPtpCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {

        EnetApp_initRxReadyPktQ(perCtxt->hRxCh, ENET_DMA_RX_CH0);
        EnetApp_initRxReadyPktQ(perCtxt->hRxPtpCh, ENET_DMA_RX_CH_PTP);
    }

     return status;
}

void EnetApp_closeDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetApp_closeRxDma((ENET_DMA_RX_CH0 + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT)),
                       perCtxt->hEnet,
                       perCtxt->coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close PTP RX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma((ENET_DMA_RX_CH_PTP + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT)),
                       perCtxt->hEnet,
                       perCtxt->coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts(perCtxt->hTxCh, &gEnetApp.txFreePktInfoQ);

    EnetApp_closeTxDma((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                       perCtxt->hEnet,
                       perCtxt->coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetApp.txFreePktInfoQ);

    /* Close PTP TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts(perCtxt->hTxPtpCh, &gEnetApp.txPtpFreePktInfoQ);

    EnetApp_closeTxDma((ENET_DMA_TX_CH_PTP + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                       perCtxt->hEnet,
                       perCtxt->coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetApp.txPtpFreePktInfoQ);

}

void EnetApp_initTxFreePktQ(EnetDma_PktQ *freePktInfoQ, uint32_t txCh)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    const uint32_t txChNumPkts[ENET_SYSCFG_TX_CHANNELS_NUM] = 
                              {
                                  [ENET_DMA_TX_CH0] = ENET_DMA_TX_CH0_NUM_PKTS, 
                                  [ENET_DMA_TX_CH_PTP] =  ENET_DMA_TX_CH_PTP_NUM_PKTS 
                              };
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to freePktInfoQ */
    for (i = 0U; i < txChNumPkts[txCh]; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(freePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() freePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(freePktInfoQ));
}

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh, uint32_t rxChIdx)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };
    const uint32_t numRxPkts[ENET_SYSCFG_RX_FLOWS_NUM] = {
                                     [ENET_DMA_RX_CH0] = ENET_DMA_RX_CH0_NUM_PKTS,
                                     [ENET_DMA_RX_CH_PTP] = ENET_DMA_RX_CH_PTP_NUM_PKTS,
                                 };

    EnetQueue_initQ(&rxFreeQ);
    EnetAppUtils_assert(rxChIdx <= ENET_ARRAYSIZE(numRxPkts));

    for (i = 0U; i < numRxPkts[rxChIdx]; i++)
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

uint32_t EnetApp_retrieveFreeTxPkts(EnetDma_TxChHandle hTxCh, EnetDma_PktQ *txPktInfoQ)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(hTxCh, &txFreeQ);
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

            EnetQueue_enq(txPktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

void EnetApp_createRxTask(EnetApp_PerCtxt *perCtxt)
{
    TaskP_Params taskParams;
    int32_t status;
    status = SemaphoreP_constructBinary(&perCtxt->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructCounting(&perCtxt->rxDoneSemObj, 0, COUNTING_SEM_COUNT);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 5U;
    taskParams.stack          = gEnetAppTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx);
    taskParams.args           = (void*)perCtxt;
    taskParams.name           = "Rx Task";
    taskParams.taskMain           = &EnetApp_rxTask;

    status = TaskP_construct(&perCtxt->rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask(EnetApp_PerCtxt *perCtxt)
{
    SemaphoreP_destruct(&perCtxt->rxSemObj);
    SemaphoreP_destruct(&perCtxt->rxDoneSemObj);
    TaskP_destruct(&perCtxt->rxTaskObj);
}

void EnetApp_rxTask(void *args)
{
    EnetApp_PerCtxt *perCtxt = (EnetApp_PerCtxt *)args;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    EthFrame *txFrame;
    uint32_t totalRxCnt = 0U;
    int32_t status = ENET_SOK;

    while ((ENET_SOK == status) && (gEnetApp.run))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&perCtxt->rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.*/
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);
        EnetQueue_initQ(&txSubmitQ);

        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(perCtxt->hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }
#if DEBUG
        EnetAppUtils_print("%s: Received %u packets\r\n", perCtxt->name, EnetQueue_getQCount(&rxReadyQ));
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
            EnetApp_retrieveFreeTxPkts(perCtxt->hTxCh, &gEnetApp.txFreePktInfoQ);

            /* Dequeue one free TX Eth packet */
            txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txFreePktInfoQ);
            if (txPktInfo != NULL)
            {
                /* Fill the TX Eth frame with test content */
                txFrame = (EthFrame *)txPktInfo->sgList.list[0].bufPtr;
                memcpy(txFrame->hdr.dstMac, rxFrame->hdr.srcMac, ENET_MAC_ADDR_LEN);
                memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U], ENET_MAC_ADDR_LEN);
                txFrame->hdr.etherType = rxFrame->hdr.etherType;

                txPktInfo->sgList.list[0].segmentFilledLen = rxPktInfo->sgList.list[0].segmentFilledLen;
                EnetAppUtils_assert(txPktInfo->sgList.list[0].segmentAllocLen >= txPktInfo->sgList.list[0].segmentFilledLen);
                memcpy(&txFrame->payload[0U],
                        &rxFrame->payload[0U],
                        rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader));

                txPktInfo->sgList.numScatterSegments = 1;
                txPktInfo->chkSumInfo = 0U;
                txPktInfo->appPriv = &gEnetApp;
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
                EnetAppUtils_print("%s: Drop due to TX pkt not available\r\n", perCtxt->name);
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
        status = EnetDma_submitTxPktQ(perCtxt->hTxCh, &txSubmitQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to submit TX pkt queue: %d\r\n", perCtxt->name, status);
        }

        EnetAppUtils_validatePacketState(&rxFreeQ,
                                            ENET_PKTSTATE_APP_WITH_FREEQ,
                                            ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        status = EnetDma_submitRxPktQ(perCtxt->hRxCh, &rxFreeQ);
        EnetAppUtils_assert(status == ENET_SOK);
        EnetAppUtils_assert(0U==EnetQueue_getQCount(&rxFreeQ));
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to submit RX pkt queue: %d\r\n", perCtxt->name, status);
        }
    }

    EnetAppUtils_print("%s: Received %u packets\r\n", perCtxt->name, totalRxCnt);

    SemaphoreP_post(&perCtxt->rxDoneSemObj);
    TaskP_exit();
}

void EnetApp_getCurrentTime(EnetApp_PerCtxt *perCtxt,
                             uint32_t *nanoseconds,
                             uint64_t *seconds)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint64_t tsVal = 0U;

    if (perCtxt != NULL)
    {
        /* Software Time stamp Push event */
        ENET_IOCTL_SET_OUT_ARGS(&prms, &tsVal);
        ENET_IOCTL(perCtxt->hEnet,
                   gEnetApp.coreId,
                   ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP,
                   &prms,
                   status);
        EnetAppUtils_assert(status == ENET_SOK);

        *nanoseconds = (uint32_t)(tsVal % (uint64_t)TIME_SEC_TO_NS);
        *seconds = tsVal / (uint64_t)TIME_SEC_TO_NS;
    }
    else
    {
        status = ENET_ENOTFOUND;
        *nanoseconds = 0U;
        *seconds = 0U;
    }
}

void EnetApp_setPortTsEventPrms(CpswMacPort_TsEventCfg *tsPortEventCfg)
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
    tsPortEventCfg->vlanLType1 = 0U;
    tsPortEventCfg->vlanLType2 = 0U;

}

int32_t EnetApp_getRxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType rxFrameType,
                                uint8_t rxPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint64_t tsVal;

    if (perCtxt != NULL)
    {
        inArgs.msgType = rxFrameType;
        inArgs.seqId   = seqId;
        inArgs.portNum = rxPort;
        inArgs.domain  = 0U;

        if (status == ENET_SOK)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
            ENET_IOCTL(perCtxt->hEnet,
                       gEnetApp.coreId,
                       ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP,
                       &prms,
                       status);
            if (status == ENET_ENOTFOUND)
            {
                tsVal = 0U;
            }
            *nanoseconds = (uint32_t)(tsVal % (uint64_t)TIME_SEC_TO_NS);
            *seconds = tsVal / (uint64_t)TIME_SEC_TO_NS;

            if (gEnetApp.enableTs)
            {
                EnetAppUtils_print("RX Timestamp: %llu\r\n", tsVal);
            }
        }
    }
    else
    {
        status = ENET_ENOTFOUND;
    }

    return status;
}

int32_t EnetApp_getTxTimestamp(EnetApp_PerCtxt *perCtxt,
                                EnetTimeSync_MsgType txFrameType,
                                uint8_t txPort,
                                uint16_t seqId,
                                uint32_t *nanoseconds,
                                uint64_t *seconds)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetTimeSync_GetEthTimestampInArgs inArgs;
    uint64_t tsVal;

    if (perCtxt != NULL)
    {
        inArgs.msgType = txFrameType;
        inArgs.seqId = seqId;
        inArgs.portNum = txPort;
        inArgs.domain  = 0U;

        if (status == ENET_SOK)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &tsVal);
            ENET_IOCTL(perCtxt->hEnet,
                                gEnetApp.coreId,
                                ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP,
                                &prms,
                                status);
            if (status == ENET_ENOTFOUND)
            {
                tsVal = 0U;
            }
            *nanoseconds = (uint32_t)(tsVal % (uint64_t)TIME_SEC_TO_NS);
            *seconds = tsVal / (uint64_t)TIME_SEC_TO_NS;
            if (gEnetApp.enableTs)
            {
                EnetAppUtils_print("TX Timestamp: %llu\r\n", tsVal);
            }
        }
    }
    else
    {
        status = ENET_ENOTFOUND;
    }

    return status;
}


int32_t EnetApp_setCpswAleClassifier(EnetApp_PerCtxt *perCtxt)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastoutArgs;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;

    memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
    memcpy(&setMcastInArgs.addr.addr[0U],
                      &peerDlyMsgMAC[0U],
                      ENET_MAC_ADDR_LEN);
    setMcastInArgs.addr.vlanId  = 0;
    setMcastInArgs.info.super = false;
    setMcastInArgs.info.numIgnBits = 0;
    setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
    setMcastInArgs.info.portMask = 3U;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(perCtxt->hEnet,
                        gEnetApp.coreId,
                        CPSW_ALE_IOCTL_ADD_MCAST,
                        &prms,
                        status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetApp_setCpswAleClassifier() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                           status);
    }

    if(status == ENET_SOK)
    {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = PTP_ETHERTYPE;
        setPolicerEntryInArgs.threadIdEn = true;
        setPolicerEntryInArgs.threadId = perCtxt->rxPtpFlowIdx;
        setPolicerEntryInArgs.peakRateInBitsPerSec = 0U;
        setPolicerEntryInArgs.commitRateInBitsPerSec = 0U;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs, &setPolicerEntryOutArgs);
        ENET_IOCTL(perCtxt->hEnet,
                   gEnetApp.coreId,
                   CPSW_ALE_IOCTL_SET_POLICER,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("EnetApp_setCpswAleClassifier() failed CPSW_ALE_IOCTL_SET_POLICER: %d\n",
                            status);
        }
    }

    return status;
}


static void EnetApp_getMsgId(EnetApp_PerCtxt* enet_perctxt,
                              uint8_t *msgId,
                              uint8_t *frame)
{
    uint32_t offset = PTP_MSG_ID_OFFSET;

    if (enet_perctxt->timeSyncConfig.protoCfg.protocol == TIMESYNC_PROT_UDP_IPV4)
    {
        offset += TIMESYNC_ANNEX_D_ANNEX_F_DIFF;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.protocol == TIMESYNC_PROT_UDP_IPV6)
    {
        offset += TIMESYNC_ANNEX_E_ANNEX_F_DIFF;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.vlanCfg.vlanType == TIMESYNC_VLAN_TYPE_SINGLE_TAG)
    {
        offset += TIMESYNC_SINGLE_TAG_VLAN_HDR_SIZE;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.vlanCfg.vlanType == TIMESYNC_VLAN_TYPE_DOUBLE_TAG)
    {
        offset += TIMESYNC_DOUBLE_TAG_VLAN_HDR_SIZE;
    }

    *msgId = *(frame + offset);
    *msgId = *msgId & 0xF;
}


static void EnetApp_getSeqId(EnetApp_PerCtxt* enet_perctxt,
                              uint16_t *seqId,
                              uint8_t *frame)
{
    uint32_t offset = PTP_SEQ_ID_OFFSET;

    if (enet_perctxt->timeSyncConfig.protoCfg.protocol == TIMESYNC_PROT_UDP_IPV4)
    {
        offset += TIMESYNC_ANNEX_D_ANNEX_F_DIFF;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.protocol == TIMESYNC_PROT_UDP_IPV6)
    {
        offset += TIMESYNC_ANNEX_E_ANNEX_F_DIFF;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.vlanCfg.vlanType == TIMESYNC_VLAN_TYPE_SINGLE_TAG)
    {
        offset += TIMESYNC_SINGLE_TAG_VLAN_HDR_SIZE;
    }

    if (enet_perctxt->timeSyncConfig.protoCfg.vlanCfg.vlanType == TIMESYNC_VLAN_TYPE_DOUBLE_TAG)
    {
        offset += TIMESYNC_DOUBLE_TAG_VLAN_HDR_SIZE;
    }

    *seqId = *(uint16_t *)(frame + offset);
    *seqId = Enet_ntohs(*seqId);
}

int32_t EnetApp_getPtpFrame(EnetApp_PerCtxt* enet_perctxt,
                             uint8_t *frame,
                             uint32_t *size,
                             uint8_t *rxPort)
{
    int32_t status = TIMESYNC_OK;
    EnetDma_Pkt *pktInfo;
    EthFrame *rxFrame;
    uint32_t rxReadyCnt =0;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    uint32_t i;

    if (enet_perctxt != NULL)
    {
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);
        status = EnetDma_retrieveRxPktQ(enet_perctxt->hRxPtpCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            //Should we bail out here?
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            //continue;
        }
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);
        if (rxReadyCnt > 0U)
        {
            /* Consume the received packets and release them */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_READYQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            rxFrame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
            *size = pktInfo->sgList.list[0].segmentFilledLen;
            memcpy(frame, rxFrame, *size);
            *rxPort = (uint8_t)pktInfo->rxPortNum;

            /* Enqueue back all the retrieved packets */
            EnetQueue_enq(&rxFreeQ, &pktInfo->node);

            for (i = 1; i < rxReadyCnt; i++)
            {
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
                EnetAppUtils_assert(pktInfo != NULL);
                EnetDma_checkPktState(&pktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_READYQ,
                                      ENET_PKTSTATE_APP_WITH_FREEQ);
                EnetQueue_enq(&rxFreeQ, &pktInfo->node);
            }

            EnetAppUtils_validatePacketState(&rxFreeQ,
                                             ENET_PKTSTATE_APP_WITH_FREEQ,
                                             ENET_PKTSTATE_APP_WITH_DRIVER);

            EnetDma_submitRxPktQ(enet_perctxt->hRxPtpCh,
                                         &rxFreeQ);
        }
        else
        {
            *size = 0U;
            status = TIMESYNC_FRAME_NOT_AVAILABLE;
        }
    }
    else
    {
        status = TIMESYNC_HANDLE_NOT_INITIALIZED;
    }
    return status;
}

int32_t EnetApp_sendPtpFrame(EnetApp_PerCtxt* enet_perctxt,
                              uint8_t *frame,
                              uint32_t size,
                              uint8_t txPort)
{
    int32_t status = TIMESYNC_OK;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    uint8_t *txFrame;
    TimeSync_TxPktInfo *pTxTsPktInfo = NULL;
    TimeSync_TxPktInfoEle *pTxTsPktInfoEle = NULL;

    if (enet_perctxt != NULL)
    {
        EnetQueue_initQ(&txSubmitQ);
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txPtpFreePktInfoQ);
        if (NULL != pktInfo)
        {
            txFrame = (uint8_t *)pktInfo->sgList.list[0].bufPtr;
            memcpy(txFrame, frame, size);
            pktInfo->appPriv = &enet_perctxt;
            pktInfo->txPortNum = (Enet_MacPort)ENET_MACPORT_NORM(txPort);
            pktInfo->sgList.list[0].segmentFilledLen = size;
            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;

            /* Save tx pkt info to re-use during notify callback */
            pTxTsPktInfo = &enet_perctxt->txTsPktInfo;
            pTxTsPktInfoEle = &pTxTsPktInfo->txTsPktInfoArr[pTxTsPktInfo->wrIdx++];
            pTxTsPktInfoEle->portNum = txPort;
            EnetApp_getMsgId(enet_perctxt,&pTxTsPktInfoEle->frameType, frame);
            EnetApp_getSeqId(enet_perctxt,&pTxTsPktInfoEle->seqId, frame);
            pTxTsPktInfo->wrIdx %= TIMESYNC_TX_PKTINFO_COUNT;

            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_FREEQ,
                                    ENET_PKTSTATE_APP_WITH_DRIVER);
            EnetQueue_enq(&txSubmitQ, &pktInfo->node);

            if (0U != EnetQueue_getQCount(&txSubmitQ))
            {
                status = EnetDma_submitTxPktQ(gEnetApp.perCtxt[0].hTxPtpCh,
                                                      &txSubmitQ);
            }
        }
    }
    else
    {
        status = TIMESYNC_HANDLE_NOT_INITIALIZED;
    }

    return status;
}


int32_t EnetApp_adjTimeSlowComp(EnetApp_PerCtxt* enet_perctxt,
                                 int32_t adjOffset,
                                 uint64_t interval)
{
    int32_t status = TIMESYNC_OK;
    Enet_IoctlPrms prms;
    EnetTimeSync_TimestampAdj adjTsInArgs;

    if (enet_perctxt != NULL)
    {
        adjTsInArgs.adjValInNsecs   = (-1 * adjOffset);
        adjTsInArgs.intervalInNsecs = interval;

        ENET_IOCTL_SET_IN_ARGS(&prms, &adjTsInArgs);
        ENET_IOCTL(enet_perctxt->hEnet, gEnetApp.coreId, ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP, &prms, status);
        EnetAppUtils_assert(status == TIMESYNC_OK);
    }
    else
    {
        status = TIMESYNC_HANDLE_NOT_INITIALIZED;
    }

    return status;
}

int32_t EnetApp_setClockTime(EnetApp_PerCtxt* enet_perctxt,
                              uint32_t nanoseconds,
                              uint64_t seconds)
{
    int32_t status = TIMESYNC_OK;
    Enet_IoctlPrms prms;

    if (enet_perctxt != NULL)
    {
        EnetTimeSync_setTimestamp timestamp = {
                .tsLoadVal = (uint64_t) (((uint64_t) seconds * (uint64_t) TIME_SEC_TO_NS) + nanoseconds), /*timestamp value to load in nanosec */
                .clkMode = 0, /*Not applicable for CPSW */
                .clkSign = 0, /*Not applicable for CPSW */
        };
        ENET_IOCTL_SET_IN_ARGS(&prms, &timestamp);
        ENET_IOCTL(enet_perctxt->hEnet, gEnetApp.coreId, ENET_TIMESYNC_IOCTL_SET_TIMESTAMP, &prms, status);
        EnetAppUtils_assert(status == TIMESYNC_OK);
    }
    else
    {
        status = TIMESYNC_HANDLE_NOT_INITIALIZED;
    }

    return status;
}



int8_t EnetApp_isPortLinkUp(EnetApp_PerCtxt* enet_perctxt,
                             uint8_t portNum)
{
    int8_t isLinkUp = FALSE;
    bool isLinkUpFlag = false;

    if (enet_perctxt != NULL)
    {
        isLinkUpFlag = EnetAppUtils_isPortLinkUp(enet_perctxt->hEnet,
                                                 gEnetApp.coreId,
                                                 (Enet_MacPort)(ENET_MACPORT_NORM(portNum)));
    }

    if (isLinkUpFlag == true)
    {
        isLinkUp = TRUE;
    }

    return isLinkUp;
}

void EnetApp_reset(EnetApp_PerCtxt* enet_perctxt)
{
    int32_t status = TIMESYNC_OK;
    EnetDma_Pkt *pktInfo;
    Enet_IoctlPrms prms;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;

    if (enet_perctxt != NULL)
    {
        EnetQueue_initQ(&rxFreeQ);
        EnetQueue_initQ(&rxReadyQ);
        /* Disable Rx & Tx Events */
        status = EnetDma_disableRxEvent(enet_perctxt->hRxCh);
        if (status != TIMESYNC_OK)
        {
            EnetAppUtils_print("EnetDma_disableRxEvent() failed: %d\n", status);
        }

        status = EnetDma_disableTxEvent(enet_perctxt->hTxCh);
        if (status != TIMESYNC_OK)
        {
            EnetAppUtils_print("EnetDma_disableTxEvent() failed: %d\n", status);
        }

        /* Clean the SW queues */
        if (status == TIMESYNC_OK)
        {
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
            while (pktInfo != NULL)
            {
                /* Consume the received packets and release them */
                EnetDma_checkPktState(&pktInfo->pktState,
                                        ENET_PKTSTATE_MODULE_APP,
                                        ENET_PKTSTATE_APP_WITH_READYQ,
                                        ENET_PKTSTATE_APP_WITH_FREEQ);

                EnetQueue_enq(&rxFreeQ, &pktInfo->node);

                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
            }

            EnetAppUtils_validatePacketState(&rxFreeQ,
                                             ENET_PKTSTATE_APP_WITH_FREEQ,
                                             ENET_PKTSTATE_APP_WITH_DRIVER);

            EnetDma_submitRxPktQ(enet_perctxt->hRxCh,
                                         &rxFreeQ);

            EnetApp_retrieveTxDonePkts();

            /* Clear Timestamp pools */
            ENET_IOCTL_SET_NO_ARGS(&prms);
            ENET_IOCTL(enet_perctxt->hEnet, gEnetApp.coreId, ENET_TIMESYNC_IOCTL_RESET, &prms, status);
            if (status != TIMESYNC_OK)
            {
                EnetAppUtils_print("ENET_TIMESYNC_IOCTL_RESET IOCTL failed: %d\n", status);
            }
        }

        /* Clear local book-keeping of Tx Info */
        if (status == TIMESYNC_OK)
        {
            memset(&enet_perctxt->txTsPktInfo, 0, sizeof(enet_perctxt->txTsPktInfo));
        }

        /* Enable Rx & Tx Events */
        if (status == TIMESYNC_OK)
        {
            status = EnetDma_enableRxEvent(enet_perctxt->hRxCh);
            if (status != TIMESYNC_OK)
            {
                EnetAppUtils_print("EnetDma_enableRxEvent() failed: %d\n", status);
            }

            status = EnetDma_enableTxEvent(enet_perctxt->hTxCh);
            if (status != TIMESYNC_OK)
            {
                EnetAppUtils_print("EnetDma_enableTxEvent() failed: %d\n", status);
            }
        }
    }

}
