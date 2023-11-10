/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef MCSPI_DMA_EDMA_H_
#define MCSPI_DMA_EDMA_H_

#include <stdint.h>
#include <drivers/edma/v0/edma.h>
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct McspiDma_EdmaArgs_s
{
    void                *drvHandle;
    /**< EDMA driver handle */
} McspiDma_EdmaArgs;

typedef struct MCSPI_DmaChConfig_s
{
    uint32_t        edmaTccRx;
    /**< EDMA TCC used for McSPI RX transfer */
    uint32_t        edmaTccTx;
    /**< EDMA TCC used for McSPI TX transfer */
    uint32_t        edmaRxChId;
    /**< EDMA Channel used for McSPI RX transfer */
    uint32_t        edmaTxChId;
    /**< EDMA Channel used for McSPI TX transfer */
    uint32_t        edmaRxParam;
    /**< EDMA Param ID used for McSPI RX transfer */
    uint32_t        edmaTxParam;
    /**< EDMA Param ID used for McSPI TX transfer */
    uint32_t        edmaDummyParam;
    /**< EDMA Param ID used for McSPI TX Dummy transfer */
    uint32_t        edmaRegionId;
    /**< EDMA Region used for McSPI transfer */
    uint32_t        edmaBaseAddr;
    /**< EDMA Base address used for McSPI transfer */
    Edma_IntrObject edmaIntrObjRx;
    /**< EDMA MCSPI RX Interrupt object */
    Edma_IntrObject edmaIntrObjTx;
    /**< EDMA MCSPI TX Interrupt object */
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */
}MCSPI_EdmaChConfig;

#ifdef __cplusplus
}
#endif

#endif /* MCSPI_DMA_EDMA_H_ */
