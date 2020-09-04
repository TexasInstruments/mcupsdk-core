/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UART_SCI_EDMA_H_
#define UART_SCI_EDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/uart.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief  Function to configure the edma channels for UART DMA mode
 *          Called from the #UART_open()
 *
 *  \param  uartHandle      #UART_Handle returned assigned in #UART_open()
 *  \param  edmaInst        edma instance to be used
 *
 *  \return #SystemP_SUCCESS if edma configuration is successfully; else error on failure
 */
int32_t UART_edmaChannelConfig(UART_Handle uartHandle, uint32_t edmaInst);

/**
 *  \brief  This function called by UART_read in DMA mode
 *
 *  \param  config     Pointer to the UART Config structure
 *  \param  object     Pointer to the UART Object structure
 *  \param  attrs      Pointer to the UART Attrbutes structure
 *  \param  trans      Transaction object passed to #UART_read()
 *
 *  \return #SystemP_SUCCESS if UART read in DMA mode is successfully; else error on failure
 */
int32_t UART_readDma(UART_Config    *config,
                    UART_Object      *object,
                    UART_Attrs const *attrs,
                    UART_Transaction *trans);

/**
 *  \brief  This function called by UART_write in DMA mode
 *
 *  \param  config     Pointer to the UART Config structure
 *  \param  object     Pointer to the UART Object structure
 *  \param  attrs      Pointer to the UART Attrbutes structure
 *  \param  trans      Transaction object passed to #UART_write()
 *
 *  \return #SystemP_SUCCESS if UART read in DMA mode is successfully; else error on failure
 */
int32_t UART_writeDma(UART_Config    *config,
                    UART_Object      *object,
                    UART_Attrs const *attrs,
                    UART_Transaction *trans);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UART_SCI_EDMA_H_ */
