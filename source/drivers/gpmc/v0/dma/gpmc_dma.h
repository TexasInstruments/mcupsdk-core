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

/**
 *  \file gpmc_dma.h
 *
 *  \brief GPMC DMA header file.
 */

#ifndef GPMC_DMA_H_
#define GPMC_DMA_H_

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup GPMC_DMA APIs for GPMC DMA mode
 *  \ingroup DRV_GPMC_MODULE
 *
 *  This module contains APIs to program and use DMA drivers available in the SoC with GPMC.
 *
 *  @{
 */

/**
 * \brief Handle to the GPMC DMA Config Object returned by \ref GPMC_dmaOpen
 */
typedef void* GPMC_DmaHandle;

/**
 * \brief Driver implementation to open a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param gpmcDmaArgs   [in] DMA specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*GPMC_dmaOpenFxn)(void *gpmcDmaArgs);

/**
 * \brief Driver implementation to close a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param GPMC_DmaHandle   [in] GPMC DMA Object handle returned from \ref GPMC_dmaOpen
 * \param gpmcDmaArgs      [in] DMA specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*GPMC_dmaCloseFxn)(GPMC_DmaHandle, void *gpmcDmaArgs);

/**
 * \brief Driver implementation to do a DMA copy using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param gpmcDmaArgs   [in] DMA specific arguments, obtained from the config
 * \param dst           [in] Destination address to which the data is to be copied
 * \param src           [in] Source address from which the data is to be copied
 * \param length        [in] Data length
 * \param fifoDrain     [in] Drain GPMC FIFO
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*GPMC_dmaCopyFxn)(void *gpmcDmaArgs, void *dst, void *src, uint32_t length, uint8_t fifoDrain);

/**
 * \brief Driver implementation callbacks
 */
typedef struct GPMC_DmaFxns_s
{
	GPMC_dmaOpenFxn    dmaOpenFxn;
	GPMC_dmaCloseFxn   dmaCloseFxn;
	GPMC_dmaCopyFxn    dmaCopyFxn;

} GPMC_DmaFxns;

/**
 * \brief GPMC DMA Configuration, these are filled by SysCfg based on the DMA driver that is selected
 */
typedef struct GPMC_DmaConfig_s
{
	GPMC_DmaFxns *fxns;
	/* Registered callbacks for a particular DMA driver. This will be set by Sysconfig depending on the DMA driver selected*/
	void *gpmcDmaArgs;
	/* Arguments specific to a DMA driver. This will be typecasted to the specific DMA driver args struct
	 * when used by the appropriate callback. This struct will be defined in the specific DMA driver header file.
	 * Allocation of this struct will be done statically using Sysconfig code generation in the example code
	 */

} GPMC_DmaConfig;

/**
 * \brief API to open an GPMC DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param index [in] Index of the DMA Config selected for this particular GPMC driver instance
 *
 * \return Handle to the GPMC DMA Config Object
 */
GPMC_DmaHandle GPMC_dmaOpen(int32_t index);

/**
 * \brief API to close an GPMC DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param index [in] Handle to the GPMC DMA Config Object returned from \ref GPMC_dmaOpen
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t GPMC_dmaClose(GPMC_DmaHandle handle);

/**
 * \brief API to do a DMA Copy using appropriate DMA Channel opened
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param handle        [in] Handle to the GPMC DMA Config Object returned from \ref GPMC_dmaOpen
 * \param dst           [in] Destination address to which the data is to be copied
 * \param src           [in] Source address from which the data is to be copied
 * \param length        [in] Data length
 * \param fifoDrain     [in] Flag to enable fifo drain for NAND devices
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t GPMC_dmaCopy(GPMC_DmaHandle handle, void* dst, void* src, uint32_t length, uint8_t fifoDrain);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* GPMC_DMA_H_ */