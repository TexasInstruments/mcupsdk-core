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
 *  \file uart_dma_dummy.c
 *
 *  \brief File containing UART Driver APIs dummy implementation for UART
 *         to avoid linking issue, as their is no UDMA support for m4 core.
 */

#ifndef UART_DMA_H_
#define UART_DMA_H_

#include <stdint.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <drivers/uart/v0/uart.h>

#ifdef __cplusplus
extern "C"
{
#endif

int32_t UART_lld_dmaInit(UARTLLD_Handle hUart, UART_DmaChConfig dmaChCfg)
{
    return UART_STATUS_SUCCESS;
}

int32_t UART_lld_dmaDeInit(UARTLLD_Handle hUart)
{
    return UART_STATUS_SUCCESS;
}

int32_t UART_lld_dmaWrite(UARTLLD_Handle hUart, const UART_Transaction  *transaction)
{
    return UART_STATUS_SUCCESS;
}

int32_t UART_lld_dmaRead(UARTLLD_Handle hUart, const UART_Transaction  *transaction)
{
    return UART_STATUS_SUCCESS;
}

int32_t UART_lld_dmaDisableChannel(UARTLLD_Handle hUart,
                                       uint32_t isChannelTx)
{
    return UART_STATUS_SUCCESS;
}

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H_ */