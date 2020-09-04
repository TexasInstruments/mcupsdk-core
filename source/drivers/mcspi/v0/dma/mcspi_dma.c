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
 *  \file mcspi_dma.c
 *
 *  \brief MCSPI DMA source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/mcspi/v0/dma/mcspi_dma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

extern MCSPI_DmaConfig gMcspiDmaConfig[];
extern uint32_t gMcspiDmaConfigNum;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

MCSPI_DmaHandle MCSPI_dmaOpen(int32_t index)
{
    MCSPI_DmaConfig *dmaConfig = NULL;

	if((gMcspiDmaConfigNum > 0) && (index >= 0))
	{
		dmaConfig = &gMcspiDmaConfig[index];
		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaOpenFxn) && (dmaConfig->mcspiDmaArgs))
		{
			int32_t status;

			status = dmaConfig->fxns->dmaOpenFxn(dmaConfig->mcspiDmaArgs);
			if(status != SystemP_SUCCESS)
			{
				dmaConfig = NULL;
			}
		}
	}

	return (MCSPI_DmaHandle)dmaConfig;
}

int32_t MCSPI_dmaChInit(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg,
                        const MCSPI_DmaChConfig *dmaChCfg)
{
	int32_t status = SystemP_SUCCESS;
    MCSPI_Config   *config;
    MCSPI_DmaHandle dmaHandle;
    DebugP_assert(NULL != handle);

    config = (MCSPI_Config *) handle;
    dmaHandle = config->object->mcspiDmaHandle;
	if(dmaHandle != NULL)
	{
		MCSPI_DmaConfig *dmaConfig = (MCSPI_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaChInitFxn))
		{
			status = dmaConfig->fxns->dmaChInitFxn(config, chCfg, dmaChCfg);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t MCSPI_dmaClose(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg)
{
	int32_t status = SystemP_SUCCESS;
    MCSPI_Config   *config;
    MCSPI_DmaHandle dmaHandle;
    DebugP_assert(NULL != handle);

    config = (MCSPI_Config *) handle;
    dmaHandle = config->object->mcspiDmaHandle;

	if(dmaHandle != NULL)
	{
		MCSPI_DmaConfig *dmaConfig = (MCSPI_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaCloseFxn))
		{
			status = dmaConfig->fxns->dmaCloseFxn(handle, chCfg);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t MCSPI_dmaTransfer(MCSPI_Object *obj,
                                MCSPI_ChObject *chObj,
                                const MCSPI_Attrs *attrs,
                                MCSPI_Transaction *transaction)
{
	int32_t status = SystemP_SUCCESS;
    MCSPI_DmaHandle dmaHandle;

    dmaHandle = obj->mcspiDmaHandle;

	if(dmaHandle != NULL)
	{
		MCSPI_DmaConfig *dmaConfig = (MCSPI_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaTransferMasterFxn))
		{
			status = dmaConfig->fxns->dmaTransferMasterFxn(obj, chObj, attrs, transaction);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t MCSPI_dmaStop(MCSPI_Object *obj,
                      const MCSPI_Attrs *attrs,
                      MCSPI_ChObject *chObj,
                      uint32_t chNum)
{
	int32_t status = SystemP_SUCCESS;
    MCSPI_DmaHandle dmaHandle;

    dmaHandle = obj->mcspiDmaHandle;

	if(dmaHandle != NULL)
	{
		MCSPI_DmaConfig *dmaConfig = (MCSPI_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaStopFxn))
		{
			status = dmaConfig->fxns->dmaStopFxn(obj, attrs, chObj, chNum);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}
