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

#ifndef GPMC_UDMA_H_
#define GPMC_UDMA_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define GPMC_DMA_UDMA_MAX_L0_XFER_SIZE (65536U)
#define GPMC_DMA_UDMA_XFER_SIZE        (64512U)

typedef struct GpmcDma_UdmaArgs_s
{
    void            *drvHandle;
    /**< UDMA driver handle */
    void            *chHandle;
    /**< UDMA channel handle */
    void            *trpdMem;
    /**< UDMA TR PD memory pointers */
    uint32_t        trpdMemSize;
    /**< Size of TR PD memory */
    void            *ringMem;
    /**< UDMA Ring memory pointers */
    uint32_t        ringMemSize;
    /**< Size of Ring Memory */
    uint32_t        ringElemCount;
    /**< Ring Element Count */
    uint32_t        localEventID;
    /**< local Event ID for BCDMA trigger */

} GpmcDma_UdmaArgs;

extern GPMC_DmaFxns gGpmcDmaUdmaFxns;

#ifdef __cplusplus
}
#endif

#endif /* GPMC_UDMA_H_ */