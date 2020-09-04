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

/**
 *  \file uart_dma.h
 *
 *  \brief UART DMA header file.
 */

#ifndef UART_DMA_H_
#define UART_DMA_H_

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/uart.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup UART_DMA APIs for UART DMA mode
 *  \ingroup DRV_UART_MODULE
 *
 *  This module contains APIs to program and use DMA drivers available in the SoC with UART.
 *
 *  @{
 */

/**
 * \brief Handle to the UART DMA Config Object returned by \ref UART_dmaOpen
 */
typedef void* UART_DmaHandle;

/**
 * \brief Driver implementation to open a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param uartHandle    [in] UART Handle
 * \param uartDmaArgs   [in] DMA specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*UART_dmaOpenFxn)(UART_Handle uartHandle, void *uartDmaArgs);

/**
 * \brief Driver implementation to do a DMA read using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param obj           [in] Pointer to UART object
 * \param attrs         [in] Pointer to UART attributes.
 * \param transaction   [in] Pointer to #UART_Transaction. This parameter can't be NULL
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*UART_dmaTransferReadFxn)(UART_Object *obj, const UART_Attrs *attrs,
                                           UART_Transaction *transaction);

/**
 * \brief Driver implementation to do a DMA write using a specific DMA driver - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param obj           [in] Pointer to UART object.
 * \param attrs         [in] Pointer to UART attributes.
 * \param transaction   [in] Pointer to #UART_Transaction. This parameter can't be NULL
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*UART_dmaTransferWriteFxn)(UART_Object *obj, const UART_Attrs *attrs,
                                            UART_Transaction *transaction);

/**
 * \brief Driver implementation to close a specific DMA driver channel - UDMA, EDMA etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new DMA driver needs to be supported.
 *
 * \param handle   [in] UART handle returned from \ref UART_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*UART_dmaCloseFxn)(UART_Handle handle);

/**
 * \brief Driver implementation to diisable a specific DMA driver channel - UDMA, EDMA etc
 *
 * \param handle        [in] UART handle returned from \ref UART_open
 * \param isChannelTx   [in] Variable to indicate if it is TX/RX Channel
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*UART_dmaDisableChannelFxn)(UART_Handle handle, uint32_t isChannelTx);

/**
 * \brief Driver implementation callbacks
 */
typedef struct UART_DmaFxns_s
{
	UART_dmaOpenFxn                dmaOpenFxn;
    UART_dmaTransferWriteFxn       dmaTransferWriteFxn;
    UART_dmaTransferReadFxn        dmaTransferReadFxn;
	UART_dmaCloseFxn               dmaCloseFxn;
	UART_dmaDisableChannelFxn      dmaDisableChannelFxn;
} UART_DmaFxns;

/**
 * \brief UART DMA Configuration, these are filled by SysCfg based on the DMA driver that is selected
 */
typedef struct UART_DmaConfig_s
{
	UART_DmaFxns *fxns;
	/** Registered callbacks for a particular DMA driver. This will be set by Sysconfig depending on the DMA driver selected */
	void *uartDmaArgs;
	/** Arguments specific to a DMA driver. This will be typecasted to the specific DMA driver args struct
	 * when used by the appropriate callback. This struct will be defined in the specific DMA driver header file.
	 * Allocation of this struct will be done statically using Sysconfig code generation in the example code
	 */
} UART_DmaConfig;

/**
 * \brief API to open an UART DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param uartHandle    [in] UART Handle
 * \param index [in] Index of the DMA Config selected for this particular UART driver instance
 *
 * \return Handle to the UART DMA Config Object
 */
UART_DmaHandle UART_dmaOpen(UART_Handle uartHandle, int32_t index);

/**
 * \brief API to close an UART DMA channel
 *
 * \param handle   [in] UART handle returned from \ref UART_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t UART_dmaClose(UART_Handle handle);

/**
 * \brief API to disable an DMA channel
 *
 * \param handle        [in] UART handle returned from \ref UART_open
 * \param isChannelTx   [in] Variable to indicate if it is TX/RX Channel
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t UART_dmaDisableChannel(UART_Handle handle, uint32_t isChannelTx);

/**
 * \brief API to write data using an UART DMA channel
 *
 * \param obj           [in] Pointer to UART object
 * \param attrs         [in] Pointer to UART attributes
 * \param transaction   [in] Pointer to #UART_Transaction. This parameter can't be NULL
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t UART_writeInterruptDma(UART_Object       *obj, const UART_Attrs *attrs,
                               UART_Transaction  *transaction);

/**
 * \brief API to read data using an UART DMA channel
 *
 * \param obj           [in] Pointer to UART object
 * \param attrs         [in] Pointer to UART attributes
 * \param transaction   [in] Pointer to #UART_Transaction. This parameter can't be NULL
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t UART_readInterruptDma(UART_Object       *obj, const UART_Attrs *attrs,
                              UART_Transaction  *transaction);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H_ */