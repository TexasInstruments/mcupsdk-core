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

#ifndef FSI_DMA_EDMA_H_
#define FSI_DMA_EDMA_H_

#include <stdint.h>
#include <drivers/edma/v0/edma.h>

#ifdef __cplusplus
extern "C"
{
#endif
typedef struct FSI_Tx_EdmaChConfig_s
{
    void            *drvHandle;
    /**< EDMA driver handle */
    uint32_t        edmaTccTx;
    /**< EDMA TCC used for FSI TX transfer */
    uint32_t        edmaTccDummy;
    /**< EDMA TCC used for FSI Dummy transfer */
    uint32_t        edmaTxChId[FSI_MAX_TX_DMA_BUFFERS];
    /**< EDMA Channel used for FSI TX transfer */
    uint32_t        edmaTxParam[FSI_MAX_TX_DMA_BUFFERS];
    /**< EDMA Param ID used for FSI TX transfer */
    uint32_t        edmaDummyParam;
    /**< EDMA Param ID used for FSI Dummy transfer */
    uint32_t        edmaRegionId;
    /**< EDMA Region used for FSI transfer */
    uint32_t        edmaBaseAddr;
    /**< EDMA Base address used for FSI transfer */
    Edma_IntrObject edmaIntrObjTx;
    /**< EDMA FSI TX Interrupt object */
    Edma_IntrObject edmaIntrObjDummy;
    /**< EDMA FSI Dummy Interrupt object */
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */
}FSI_Tx_EdmaChConfig;

typedef struct FSI_Rx_EdmaChConfig_s
{
    void            *drvHandle;
    /**< EDMA driver handle */
    uint32_t        edmaTccRx;
    /**< EDMA TCC used for FSI RX transfer */
    uint32_t        edmaTccDummy;
    /**< EDMA TCC used for FSI Dummy transfer */
    uint32_t        edmaRxChId[FSI_MAX_RX_DMA_BUFFERS];
    /**< EDMA Channel used for FSI RX transfer */
    uint32_t        edmaRxParam[FSI_MAX_RX_DMA_BUFFERS];
    /**< EDMA Param ID used for FSI RX transfer */
    uint32_t        edmaDummyParam;
    /**< EDMA Param ID used for FSI Dummy transfer */
    uint32_t        edmaRegionId;
    /**< EDMA Region used for FSI transfer */
    uint32_t        edmaBaseAddr;
    /**< EDMA Base address used for FSI transfer */
    Edma_IntrObject edmaIntrObjRx;
    /**< EDMA FSI RX Interrupt object */
    Edma_IntrObject edmaIntrObjDummy;
    /**< EDMA FSI Dummy Interrupt object */
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */
}FSI_Rx_EdmaChConfig;

#ifdef __cplusplus
}
#endif

#endif /* FSI_DMA_EDMA_H_ */