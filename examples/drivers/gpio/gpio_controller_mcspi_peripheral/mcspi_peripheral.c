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

/* This example demonstrates the McSPI Peripheral TX & RX operation configured
 * in callback, interrupt mode of operation.
 *
 * It receives a clock from the controller(R5FSS0_0) and begins a full-duplex
 * spi transfer.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */

#include <kernel/dpl/DebugP.h>
#include "am263x-cc/r5fss0-1_nortos/mcspi_peripheral.h"

/* Function definitions */
static inline void App_bufferFill();

/* Semaphore to indicate Tx/Rx completion used in callback api's */
static SemaphoreP_Object gMcspiTransferDoneSem;
void mcspi_transfer_callback(MCSPI_Handle handle, MCSPI_Transaction *trans);

void *mcspi_peripheral_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;

    Drivers_open();
    Board_driversOpen();

	DebugP_log("MCSPI peripheral operation started...\r\n");

	/* Memfill buffers */
    App_bufferFill();

	/* Create binary semaphore */
    status = SemaphoreP_constructBinary(&gMcspiTransferDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Initiate transfer */
    MCSPI_Transaction_init(&spiTransaction);
    spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
    spiTransaction.dataSize = APP_GPIOSPI_DATASIZE;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_GPIOSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;

	/* Begin Transfer::Start peripheral to get ready */
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);

    /* Signify that peripheral is ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Wait for transfer completion */
    SemaphoreP_pend(&gMcspiTransferDoneSem, SystemP_WAIT_FOREVER);

    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        DebugP_assert(FALSE); /* MCSPI transfer failed!! */
    }
    else
    {
        /* Compare data */
        for(i = 0U; i < APP_GPIOSPI_MSGSIZE; i++)
        {
            if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
            DebugP_log("\r\nPERIPHERAL | \t 0x%x \t | \t 0x%x\r\n", gMcspiTxBuffer[i], gMcspiRxBuffer[i]);
        }
    }

    SemaphoreP_destruct(&gMcspiTransferDoneSem);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return NULL;
}

void mcspi_transfer_callback(MCSPI_Handle handle, MCSPI_Transaction *trans)
{
    DebugP_assertNoLog(MCSPI_TRANSFER_COMPLETED == trans->status);
    SemaphoreP_post(&gMcspiTransferDoneSem);

    return;
}

static inline void App_bufferFill()
{
	uint32_t 	i;

	for(i=0; i<APP_GPIOSPI_MSGSIZE; i++)
    {
#ifdef APP_GPIOSPI_8BIT
        gMcspiTxBuffer[i] = i + 0xAA;
#else
        gMcspiTxBuffer[i] = i + 0xAAAAAAAA;
#endif
        gMcspiRxBuffer[i] = 0;
    }
}
