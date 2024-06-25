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

GPMC_DmaHandle GPMC_dmaOpen(int32_t index)
{
	GPMC_DmaConfig *config = NULL;

	if((gGpmcDmaConfigNum > 0) && (index >= 0))
	{
		config = &gGpmcDmaConfig[index];
		if((config->fxns) && (config->fxns->dmaOpenFxn) && (config->gpmcDmaArgs))
		{
			int32_t status;

			status = config->fxns->dmaOpenFxn(config->gpmcDmaArgs);
			if(status != SystemP_SUCCESS)
			{
				config = NULL;
			}
		}
	}

	return (GPMC_DmaHandle)config;
}

int32_t GPMC_dmaClose(GPMC_DmaHandle handle)
{
	int32_t status = SystemP_SUCCESS;

	if(handle != NULL)
	{
		GPMC_DmaConfig *config = (GPMC_DmaConfig *)handle;

		if((config->fxns) && (config->fxns->dmaCloseFxn))
		{
			status = config->fxns->dmaCloseFxn(handle, config->gpmcDmaArgs);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t GPMC_dmaCopy(GPMC_DmaHandle handle, void* dst, void* src, uint32_t length, uint8_t fifoDrain)
{
	int32_t status = SystemP_SUCCESS;

	if(handle != NULL)
	{
		GPMC_DmaConfig *config = (GPMC_DmaConfig *)handle;

		if((config->fxns) && (config->fxns->dmaCopyFxn))
		{
			status = config->fxns->dmaCopyFxn(config->gpmcDmaArgs, dst, src, length, fifoDrain);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}