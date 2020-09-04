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

#ifndef UART_DMA_UDMA_H_
#define UART_DMA_UDMA_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct UartDma_UdmaArgs_s
{
    void            *drvHandle;
    /**< UDMA driver handle */
    void            *txChHandle;
    /**< UDMA channel tx handle */
    void            *rxChHandle;
    /**< UDMA channel rx handle */
    void            *cqTxEvtHandle;
    /**< UDMA cq tx event handle */
    void            *cqRxEvtHandle;
    /**< UDMA cq rx event handle */
    void            *txHpdMem;
    /**< UDMA TX HPD memory pointers */
    void            *rxHpdMem;
    /**< UDMA RX HPD memory pointers */
    uint32_t        hpdMemSize;
    /**< Size of TR PD memory */
    void            *txRingMem;
    /**< UDMA TX Ring memory pointers */
    void            *rxRingMem;
    /**< UDMA RX Ring memory pointers */
    uint32_t        ringMemSize;
    /**< Size of Ring Memory */
    uint32_t        ringElemCnt;
    /**< Ring Element Count */
    uint32_t        isOpen;
    /**< Flag to indicate whether the DMA instance is opened already */
}UartDma_UdmaArgs;

extern UART_DmaFxns gUartDmaUdmaFxns;

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_UDMA_H_ */