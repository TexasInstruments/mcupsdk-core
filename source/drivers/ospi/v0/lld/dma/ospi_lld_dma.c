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

/**
 *  \file ospi_dma.c
 *
 *  \brief OSPI DMA source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

extern OSPI_DmaConfig gOspiDmaConfig[];
extern uint32_t gOspiDmaConfigNum;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

int32_t OSPI_dmaOpen(int32_t index)
{
	OSPI_DmaConfig *config = NULL;
	int32_t status = SystemP_SUCCESS;

	if((gOspiDmaConfigNum > 0U) && (index >= 0))
	{
		config = &gOspiDmaConfig[index];
		if((config->fxns) && (config->fxns->dmaOpenFxn) && (config->ospiDmaArgs))
		{
			status = config->fxns->dmaOpenFxn(config->ospiDrvHandle);
			if(status != SystemP_SUCCESS)
			{
				config = NULL;
			}
		}
	}

	// return (OSPI_DmaHandle)config;
	return status;
}

int32_t OSPI_dmaClose(OSPI_DmaHandle handle)
{
	int32_t status = SystemP_SUCCESS;

	if(NULL != handle)
	{
		OSPI_DmaConfig *config = (OSPI_DmaConfig *)handle;

		if((config->fxns) && (config->fxns->dmaCloseFxn))
		{
			status = config->fxns->dmaCloseFxn(config->ospiDrvHandle);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t OSPI_dmaCopy(OSPI_DmaHandle handle, void* dst, void* src, uint32_t length, uint32_t timeout)
{
	int32_t status = SystemP_SUCCESS;

	if(NULL != handle)
	{
		OSPI_DmaConfig *config = (OSPI_DmaConfig *)handle;

		if((config->fxns) && (config->fxns->dmaCopyFxn))
		{
			status = config->fxns->dmaCopyFxn(config->ospiDrvHandle, dst, src, length, timeout);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t OSPI_isDmaInterruptEnabled(OSPI_DmaHandle handle)
{
	int32_t retVal = SystemP_SUCCESS;

	if(NULL != handle)
	{
		OSPI_DmaConfig *config = (OSPI_DmaConfig *)handle;

		if((config->fxns) && (config->fxns->dmaItrStatusFxn))
		{
			retVal = config->fxns->dmaItrStatusFxn(config->ospiDrvHandle);
		}
	}
	else
	{
		retVal = SystemP_FAILURE;
	}

	return retVal;

}