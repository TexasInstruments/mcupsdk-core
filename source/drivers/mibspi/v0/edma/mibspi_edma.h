/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef MIBSPI_EDMA_H_
#define MIBSPI_EDMA_H_

#include <stdint.h>
#include <drivers/mibspi.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  @brief  Function to open DMA handle, and to get the base address the region ID
 *
 *  Function invoked during MIBSPI_open inside the driver to get the DMA handle
 *
 *  \param  handle       mibspi handle.
 *  \param  edmaInst     EDMA instance used for MIBSPI transfers.
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 */
int32_t MIBSPI_edmaOpen(MIBSPI_Handle handle, uint32_t edmaInst);

/**
 *  \brief  Function to configure DMA channel pair for the specified DMA line
 *
 *  Function invoked during MIBSPI_open inside the driver to configure the
 *  Tx and Rx DMA channel connected to the specified DMA request line
 *
 *  @param  handle      A MIBSPI_Handle
 *  @param  dmaReqLine  DMA request line
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 *
 */
int32_t MIBSPI_edmaAllocChResource(MIBSPI_Handle handle, uint32_t dmaReqLine);

/**
 *  \brief  Function to configure the DMA transfer for the specified transaction
 *
 *  Function invoked during MIBSPI_transfer inside the driver to configure the
 *  Tx and Rx DMA for the transaction
 *
 *  @param  handle      A MIBSPI_Handle
 *  @param  xferInfo    Tx and Rx Transaction transfer info
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 *
 */
int32_t MIBSPI_edmaTransfer(MIBSPI_Handle handle, MIBSPI_DMAXferInfo *xferInfo);

/**
 *  @brief  Function to free the DMA channels pair for the specified DMA line
 *
 *  Function invoked during MIBSPI_close inside the driver to uninitialize the
 *  Tx and Rx DMA channels associated with the DMA request line
 *
 *  @param  handle      A MIBSPI_Handle
 *  @param  dmaReqLine  DMA request line
 * 
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 *
 */
int32_t MIBSPI_edmaFreeChResource(const MIBSPI_Handle handle,  uint32_t dmaReqLine);

#ifdef __cplusplus
}
#endif

#endif /* MIBSPI_EDMA_H_ */
