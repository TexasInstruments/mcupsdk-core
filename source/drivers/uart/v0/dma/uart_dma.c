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
 *  \file uart_dma.c
 *
 *  \brief UART DMA abstract source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/uart/v0/dma/uart_dma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

extern UART_DmaConfig gUartDmaConfig[];
extern uint32_t gUartDmaConfigNum;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */
UART_DmaHandle UART_dmaOpen(UART_Handle uartHandle, int32_t index)
{
    UART_DmaConfig *dmaConfig = NULL;

	if((gUartDmaConfigNum > 0) && (index >= 0))
	{
		dmaConfig = &gUartDmaConfig[index];
		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaOpenFxn) && (dmaConfig->uartDmaArgs))
		{
			int32_t status;

			status = dmaConfig->fxns->dmaOpenFxn(uartHandle, dmaConfig->uartDmaArgs);
			if(status != SystemP_SUCCESS)
			{
				dmaConfig = NULL;
			}
		}
	}

	return (UART_DmaHandle)dmaConfig;
}

int32_t UART_dmaClose(UART_Handle handle)
{
	int32_t status = SystemP_SUCCESS;
    UART_Config   *config;
    UART_DmaHandle dmaHandle;
    DebugP_assert(NULL != handle);

    config = (UART_Config *) handle;
    dmaHandle = config->object->uartDmaHandle;

	if(dmaHandle != NULL)
	{
		UART_DmaConfig *dmaConfig = (UART_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaCloseFxn))
		{
			status = dmaConfig->fxns->dmaCloseFxn(handle);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t UART_dmaDisableChannel(UART_Handle handle, uint32_t isChannelTx)
{
	int32_t status = SystemP_SUCCESS;
    UART_Config   *config;
    UART_DmaHandle dmaHandle;
    DebugP_assert(NULL != handle);

    config = (UART_Config *) handle;
    dmaHandle = config->object->uartDmaHandle;

	if(dmaHandle != NULL)
	{
		UART_DmaConfig *dmaConfig = (UART_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaDisableChannelFxn))
		{
			status = dmaConfig->fxns->dmaDisableChannelFxn(handle, isChannelTx);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t UART_readInterruptDma(UART_Object       *obj, const UART_Attrs   *attrs,
                              UART_Transaction  *transaction)
{
	int32_t status = SystemP_SUCCESS;
    UART_DmaHandle dmaHandle;

    dmaHandle = obj->uartDmaHandle;
	if(dmaHandle != NULL)
	{
		UART_DmaConfig *dmaConfig = (UART_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaTransferReadFxn))
		{
			status = dmaConfig->fxns->dmaTransferReadFxn(obj, attrs, transaction);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t UART_writeInterruptDma(UART_Object       *obj, const UART_Attrs   *attrs,
                               UART_Transaction  *transaction)
{
	int32_t status = SystemP_SUCCESS;
    UART_DmaHandle dmaHandle;

    dmaHandle = obj->uartDmaHandle;
	if(dmaHandle != NULL)
	{
		UART_DmaConfig *dmaConfig = (UART_DmaConfig *)dmaHandle;

		if((dmaConfig->fxns) && (dmaConfig->fxns->dmaTransferWriteFxn))
		{
			status = dmaConfig->fxns->dmaTransferWriteFxn(obj, attrs, transaction);
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}
