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

#ifndef OSPI_EDMA_LLD_H_
#define OSPI_EDMA_LLD_H_

#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ospi.h>
#include <drivers/edma.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief OSPI EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for OSPI transfer.
 *
 */
typedef struct
{
    uint32_t edmaTcc;
    /**< EDMA TCC used for QSPI transfer */
    uint32_t edmaChId;
    /**< EDMA Channel used for QSPI transfer */
    uint32_t edmaChainChId;
    /**< EDMA Chained Channel used for QSPI transfer */
    uint32_t edmaParam;
    /**< EDMA Param ID used for QSPI transfer */
    uint32_t edmaChainParam;
    /**< EDMA Param ID used for Chained channel in QSPI transfer */
    uint32_t edmaRegionId;
    /**< EDMA Region used for QSPI transfer */
    uint32_t edmaBaseAddr;
    /**< EDMA Base address used for QSPI transfer */
    uint32_t isIntEnabled;
    /**< EDMA Interrupt enabled status */    
    uint32_t edmaInst;
    /**< EDMA Instance */    
    Edma_IntrObject edmaIntrObj;
    /**< EDMA Interrupt object */
} OspiDma_EdmaArgs;

extern OSPI_DmaFxns gOspiDmaEdmaFxns;

/**
 * \brief API to open an OSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param index [in] Index of the DMA Config selected for this particular OSPI driver instance
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_edmaInit(OSPI_DmaHandle handle);

/**
 * \brief API to close an OSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param index [in] Handle to the OSPI DMA Config Object returned from \ref OSPI_dmaOpen
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_edmaDeInit(OSPI_DmaHandle handle);

/**
 * \brief API to do a DMA Copy using appropriate DMA Channel opened
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param handle        [in] Handle to the OSPI DMA Config Object returned from \ref OSPI_dmaOpen
 * \param dst           [in] Destination address to which the data is to be copied
 * \param src           [in] Source address from which the data is to be copied
 * \param length        [in] Data length
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_edmaCopy(OSPI_DmaHandle handle, void* dst, void* src, uint32_t length,uint32_t timeout);

/**
 * \brief API to close an OSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param index [in] Handle to the OSPI DMA Config Object returned from \ref OSPI_dmaOpen
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_edmaItrStatus(OSPI_DmaHandle handle);


#ifdef __cplusplus
}
#endif

#endif /* OSPI_EDMA_LLD_H_ */