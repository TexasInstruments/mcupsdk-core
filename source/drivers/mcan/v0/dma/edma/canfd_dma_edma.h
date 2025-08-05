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

#ifndef CANFD__DMA_EDMA_H_
#define CANFD__DMA_EDMA_H_

#include <stdint.h>
#include <drivers/edma/v0/edma.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct CANFD_EdmaArgs_s
{
    EDMA_Handle               drvHandle;
    /**< EDMA driver handle */
} CANFD_EdmaArgs;

typedef struct CANFD_EdmaChConfig_s
{
    uint32_t        edmaRegionId;
    /**< EDMA Region used for MCAN transfer */
    uint32_t        edmaBaseAddr;
    /**< EDMA Base address used for MCAN transfer */
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */

    uint32_t        edmaRxChAlloc;
    /**< EDMA Rx Event Allocation. Application should set the mapped dma channel numbers.
     *   Driver will allocate from 0 to MCAN_MAX_RX_DMA_BUFFERS while allocating the channel object.
     *   the dma channels should be mapped to MCANSSn_FE_0 to MCANSSn_FE_n respectively.
    */
    uint32_t        edmaRxChId[MCAN_MAX_RX_DMA_BUFFERS];
    /**< EDMA Channel used for MCAN RX */
    uint32_t        edmaRxTcc[MCAN_MAX_RX_DMA_BUFFERS];
    /**< EDMA TCC used for MCAN RX */
    uint32_t        edmaRxParam[MCAN_MAX_RX_DMA_BUFFERS];
    /**< EDMA Param ID used for MCAN RX */
    Edma_IntrObject edmaIntrObjRx[MCAN_MAX_RX_DMA_BUFFERS];
    /**< EDMA MCAN RX Interrupt object */

    uint32_t        edmaTxChAlloc;
    /**< EDMA Tx Event Allocation  Application should set the mapped dma channel numbers.
     *   Driver will allocate from 0 to MCAN_MAX_TX_DMA_BUFFERS while allocating the channel object.
     *   The dma channels should be mapped from MCANSSn_TX_DMA_0 to MCANSSn_TX_DMA_n respectively. */
    uint32_t        edmaTxChId[MCAN_MAX_TX_DMA_BUFFERS];
    /**< EDMA Channel used for MCAN TX */
    uint32_t        edmaTxTcc[MCAN_MAX_TX_DMA_BUFFERS];
    /**< EDMA TCC used for MCAN TX */
    uint32_t        edmaTxParam[MCAN_MAX_TX_DMA_BUFFERS];
    /**< EDMA Param ID used for MCAN TX */
    Edma_IntrObject edmaIntrObjTx[MCAN_MAX_TX_DMA_BUFFERS];
    /**< EDMA MCAN TX Interrupt object */
}CANFD_EdmaChConfig;

/** <Opaque handle to a CAN FD DMA channel configuration structure. */
typedef struct CANFD_EdmaChConfig_s *CANFD_DmaChConfig;
/** <Opaque handle to a CAN FD DMA configuration structure. */
typedef EDMA_Config  *CANFD_DmaHandle;

#ifdef __cplusplus
}
#endif

#endif /* CANFD__DMA_EDMA_H_ */
