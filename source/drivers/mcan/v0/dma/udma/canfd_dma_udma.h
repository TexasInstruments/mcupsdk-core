/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef CANFD__DMA_UDMA_H_
#define CANFD__DMA_UDMA_H_

#include <stdint.h>
#include <drivers/udma.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct CANFD_UdmaArgs_s
{
    Udma_DrvHandle       drvHandle;
    /**< UDMA driver handle */
} CANFD_UdmaArgs;

typedef struct CANFD_UdmaChConfig_s
{
    Udma_ChHandle         txChHandle[MCAN_MAX_TX_DMA_BUFFERS];
    /**< UDMA channel tx handle */
    Udma_ChHandle         rxChHandle[MCAN_MAX_RX_DMA_BUFFERS];
    /**< UDMA channel rx handle */
    Udma_EventHandle      cqTxEvtHandle[MCAN_MAX_TX_DMA_BUFFERS];
    /**< UDMA cq tx event handle */
    Udma_EventHandle      cqRxEvtHandle[MCAN_MAX_RX_DMA_BUFFERS];
    /**< UDMA cq rx event handle */
    void            *txHpdMem[MCAN_MAX_TX_DMA_BUFFERS];
    /**< UDMA TX HPD memory pointers */
    void            *rxHpdMem[MCAN_MAX_RX_DMA_BUFFERS];
    /**< UDMA RX HPD memory pointers */
    uint32_t        hpdMemSize;
    /**< Size of TR PD memory */
    void            *txRingMem[MCAN_MAX_TX_DMA_BUFFERS];
    /**< UDMA TX Ring memory pointers */
    void            *rxRingMem[MCAN_MAX_RX_DMA_BUFFERS];
    /**< UDMA RX Ring memory pointers */
    uint32_t        ringMemSize;
    /**< Size of Ring Memory */
    uint32_t        ringElemCnt;
    /**< Ring Element Count */
    uint32_t        rxEvtNum[MCAN_MAX_TX_DMA_BUFFERS];
    /**< UDMA Event number used for Rx */
    uint32_t        txEvtNum[MCAN_MAX_RX_DMA_BUFFERS];
    /**< UDMA Event number used for Tx */
    uint32_t        udmaTxChAlloc;
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */
}CANFD_UdmaChConfig;

/** <Opaque handle to a CAN FD DMA channel configuration structure. */
typedef struct CANFD_UdmaChConfig_s *CANFD_DmaChConfig;
/** <Opaque handle to a CAN FD DMA configuration structure. */
typedef struct Udma_DrvObjectInt_t  *CANFD_DmaHandle;

#ifdef __cplusplus
}
#endif

#endif /* CANFD__DMA_UDMA_H_ */
