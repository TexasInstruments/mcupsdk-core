/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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
 *  \file gpmc_dma.c
 *
 *  \brief GPMC DMA source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

extern GPMC_DmaConfig gGpmcDmaConfig[];
extern uint32_t gGpmcDmaConfigNum;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

Gpmc_DmaArgs* GPMC_dmaOpen(int32_t index)
{
	GPMC_DmaConfig *config = NULL;
	Gpmc_DmaArgs *dmaArgs = NULL;

    if((gGpmcDmaConfigNum > 0) && (index >= 0))
    {
        config = &gGpmcDmaConfig[index];
		if(config != NULL)
		{
			if(config->gpmcDmaArgs)
			{
				int32_t status = SystemP_SUCCESS;
#if defined(DMA_VERSION_GPMC_UDMA)
				status = GpmcDma_udmaOpen(config->gpmcDmaArgs);
#endif
				if(status == SystemP_SUCCESS)
				{
					dmaArgs = config->gpmcDmaArgs;
				}
			}
		}
    }

    return dmaArgs;
}

int32_t GPMC_dmaClose(Gpmc_DmaArgs *gpmcDmaArgs)
{
	int32_t status = SystemP_SUCCESS;

	if(gpmcDmaArgs != NULL)
	{
#if defined(DMA_VERSION_GPMC_UDMA)
		status = GpmcDma_udmaClose(gpmcDmaArgs);
#endif
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t GPMC_dmaCopy(Gpmc_DmaArgs *gpmcDmaArgs, void *dst, void *src, uint32_t length, uint8_t fifoDrain)
{
	int32_t status = SystemP_SUCCESS;

	if(gpmcDmaArgs != NULL)
	{
#if defined(DMA_VERSION_GPMC_UDMA)
		status = GpmcDma_udmaCopy(gpmcDmaArgs, (uint32_t*) dst, (uint32_t*) src, length, fifoDrain);
#endif
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}