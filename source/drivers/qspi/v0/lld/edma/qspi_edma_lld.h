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

#ifndef QSPI_EDMA_H_
#define QSPI_EDMA_H_

#include <stdint.h>

#include <drivers/edma/v0/edma.h>
#include <drivers/qspi/v0/lld/qspi_lld.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief QSPI EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for QSPI transfer.
 *
 */
typedef struct QSPI_EdmaParams_s
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
    Edma_IntrObject edmaIntrObj;
    /**< EDMA Interrupt object */
} QSPI_EdmaParams;

/**
 *  \brief  Function to initialize EDMA before transfers.
 *
 *  \param  qspiHandle   #QSPILLD_Handle returned from #QSPI_open().
 *
 *  \return #SystemP_SUCCESS if configured successfully; else error on failure
 */
int32_t QSPI_edmaChannelConfig(QSPILLD_Handle qspiHandle);

/**
 *  \brief  Function to transfer QSPI data using EDMA.
 *
 *  \param  dst         Destination address of DMA transfer.
 *  \param  src         Source address for DMA transfer.
 *  \param  length      Number of bytes to be transferred.
 *  \param  qspiHandle  #QSPILLD_Handle returned from #QSPI_open().
 *  \param  timeout     timeout parameter
 *
 */
void QSPI_edmaTransfer(void* dst, void* src, uint32_t length,
                       QSPILLD_Handle qspiHandle, uint32_t timeout);

/**
 *  \brief  Function to free EDMA resources.
 *
 *  \param  qspiHandle   #QSPILLD_Handle returned from #QSPI_open().
 *
 *  \return #SystemP_SUCCESS if resources freed successfully; else error on failure
 */
int32_t QSPI_edmaChannelFree(QSPILLD_Handle qspiHandle);

#ifdef __cplusplus
}
#endif

#endif /* QSPI_EDMA_H_ */