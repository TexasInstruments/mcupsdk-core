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

/*!
 * \file  cpsw_fast_startup_dataflow.c
 *
 * \brief This file contains the implementation of the APIs for data flow for Enet Fast Startup example
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "cpsw_fast_startup_common.h"
#include "cpsw_fast_startup_dataflow.h"
#include "enet_profiler.h"
#include "ti_enet_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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


int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    EnetApp_GetDmaHandleInArgs     rxInArgs;
    EnetApp_GetRxDmaHandleOutArgs  rxChInfo; 

    /* Open the TX channel */
    txInArgs.cbArg   = &gEnetApp.appEvents;
    txInArgs.notifyCb = EnetApp_postTxEvent;

    EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                           &txInArgs,
                           &txChInfo);

    gEnetApp.hTxCh   = txChInfo.hTxCh;

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    if (gEnetApp.hTxCh == NULL)
    {
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        EnetAppUtils_assert(gEnetApp.hTxCh != NULL);
    }
    else
    {
        status = EnetDma_enableTxEvent(gEnetApp.hTxCh);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        rxInArgs.notifyCb = EnetApp_postRxEvent;
        rxInArgs.cbArg   = &gEnetApp.appEvents;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                               &rxInArgs,
                               &rxChInfo);
        gEnetApp.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetApp.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);

        if (gEnetApp.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetApp.hRxCh != NULL);
        }
        else
        {
            /* Submit all ready RX buffers to DMA.*/
            EnetApp_initRxReadyPktQ(gEnetApp.hRxCh);
        }
    }

     return status;
}

void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

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

void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
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

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetApp.txFreePktInfoQ));
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

uint32_t EnetApp_retrieveFreeTxPkts(void)
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
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

void EnetApp_postTxEvent(void* pArg)
{
    PROFILE_TIME(ClockP_getTimeUsec());
    EventP_Object* pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_TXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post TX Event\r\n");
        EnetAppUtils_assert(false);
    }

}

void EnetApp_postRxEvent(void* pArg)
{
    PROFILE_TIME(ClockP_getTimeUsec());
    EventP_Object* pEvent = (EventP_Object*) pArg;
    const int32_t status = EventP_setBits(pEvent, AppEventId_RXPKT);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to post Rx Event\r\n");
        EnetAppUtils_assert(false);
    }
}

void EnetApp_preparePktQ(EnetDma_PktQ* pktQueue)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *txPktInfo;
    EthFrame *frame;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};
    uint8_t localAddr[ENET_MAC_ADDR_LEN] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    
    /* App ready to send a packet after link up*/
    EnetQueue_initQ(&txSubmitQ);
    
    /* Dequeue one free TX Eth packet */
    txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txFreePktInfoQ);
    EnetAppUtils_assert(txPktInfo != NULL);
    
    /* Fill the TX Eth frame with test content */
    frame = (EthFrame *)txPktInfo->sgList.list[0].bufPtr;
    memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
    memcpy(frame->hdr.srcMac, localAddr, ENET_MAC_ADDR_LEN);
    frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
    memset(&frame->payload[0U], (uint8_t)(0xA5), 500);

    txPktInfo->sgList.list[0].segmentFilledLen = 500U + sizeof(EthFrameHeader);
    EnetAppUtils_assert(txPktInfo->sgList.list[0].segmentAllocLen >= txPktInfo->sgList.list[0].segmentFilledLen);

    txPktInfo->sgList.numScatterSegments = 1;
    txPktInfo->chkSumInfo = 0U;
    txPktInfo->appPriv = &gEnetApp;
    txPktInfo->txPortNum = ENET_MAC_PORT_1;
    txPktInfo->tsInfo.enableHostTxTs = false;

    EnetDma_checkPktState(&txPktInfo->pktState,
                          ENET_PKTSTATE_MODULE_APP,
                          ENET_PKTSTATE_APP_WITH_FREEQ,
                          ENET_PKTSTATE_APP_WITH_DRIVER);

    /* Enqueue the packet for later transmission */
    EnetQueue_enq(pktQueue, &txPktInfo->node);

}

void EnetApp_sendPktQ(EnetDma_PktQ* pPktQ)
{

    const int32_t status = EnetDma_submitTxPktQ(gEnetApp.hTxCh, pPktQ);

    /* Retrieve TX free packets */
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetDma_submitTxPktQ() failed to submit pkts: %d\r\n",
                           status);
    }
}

void EnetApp_handleRxPkt()
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *rxPktInfo;
    EthFrame *rxFrame;
    int32_t status;
    
    PROFILE_TIME(ClockP_getTimeUsec());
    
    EnetQueue_initQ(&rxReadyQ);
    
    /* Get the packets received so far */
    status = EnetDma_retrieveRxPktQ(gEnetApp.hRxCh, &rxReadyQ);
    if (status != ENET_SOK)
    {
        /* Should we bail out here? */
        EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
    }
      
    /* Consume the received packets and send them back */
    rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
    if(rxPktInfo != NULL)
    {
        rxFrame = (EthFrame *)rxPktInfo->sgList.list[0].bufPtr;
        if(rxFrame == NULL)
        {
             EnetAppUtils_print("Received frame is NULL: %d\r\n");
        }
        EnetDma_checkPktState(&rxPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_DRIVER,
                                ENET_PKTSTATE_APP_WITH_READYQ);
                                
        /* RX packet is ready for consumption and processing*/
        
    }
    else
    {
        EnetAppUtils_print("%s: RX is null \r\n", &gEnetApp.name);
    }
    /* Posting the termination event after first packet reception, comment the below code to stop termination*/
    status = EventP_setBits(&gEnetApp.appEvents, AppEventId_TERMINATE);

}
