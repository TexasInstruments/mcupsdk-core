/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

/* This example demonstrates the McSPI RX and TX operation configured
 * in blocking, interrupt mode of operation with External loopback connection
 * of 2 McSPI instances controlled by different cores - one in controller and 
 * other in peripheral mode. This is the Periperal core.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * to the peripheral and then receives the same in RX mode from controller.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 * 
 * Please refer to Example documentation for External connection details.
 */

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/tistdtypes.h>

#define APP_MCSPI_MSGSIZE                   (128U)

uint8_t gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t gMcspiRxBuffer[APP_MCSPI_MSGSIZE];

void *mcspi_peripheral_transfer_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCSPI] External Loopback example started in Peripheral mode...\r\n");

    /* Memfill buffers */
    for(i = 0U; i < APP_MCSPI_MSGSIZE; i++)
    {
        gMcspiTxBuffer[i] = i;
        gMcspiRxBuffer[i] = 0U;
    }

    /* indicate mcspi master that peripheral is ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Initiate transfer */
    MCSPI_Transaction_init(&spiTransaction);
    spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
    spiTransaction.dataSize  = 8;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE / (spiTransaction.dataSize/8);
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);

    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        DebugP_assert(FALSE); /* MCSPI transfer failed!! */
    }
    else
    {
        /* Compare data */
        for(i = 0U; i < APP_MCSPI_MSGSIZE; i++)
        {
            if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d in Peripheral mode.\r\n", i);
                break;
            }
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("Successfully received and sent data in Peripheral mode!! \r\n");
    }
    else
    {
        DebugP_log("Some tests have failed in Peripheral mode!!\r\n");
    }

    Board_driversClose();
    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    /* Drivers_close(); */

    return NULL;
}