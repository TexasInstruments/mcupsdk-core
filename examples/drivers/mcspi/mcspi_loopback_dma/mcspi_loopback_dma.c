/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 * in DMA mode of operation.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * and then receives the same in RX mode. Internal pad level loopback mode
 * is enabled to receive data.
 * To enable internal pad level loopback mode, D0 pin is configured to both
 * TX Enable as well as RX input pin in the SYSCFG.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/tistdtypes.h>

#define APP_MCSPI_MSGSIZE                   (128U)
#define APP_MCSPI_TRANSFER_LOOPCOUNT        (10U)

uint8_t gMcspiTxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t gMcspiRxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void *mcspi_loopback_dma_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, j;
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    uint32_t            startTimeInUSec, elapsedTimeInUsecs;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCSPI] Loopback example DMA mode started ...\r\n");

    /* Memfill buffers */
    for(i = 0U; i < APP_MCSPI_MSGSIZE; i++)
    {
        gMcspiTxBuffer[i] = i + 1U;
        gMcspiRxBuffer[i] = 0U;
    }

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer[0U], sizeof(gMcspiTxBuffer), CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);

    /* Initiate transfer */
    MCSPI_Transaction_init(&spiTransaction);
    spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
    spiTransaction.dataSize = 8U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE / (spiTransaction.dataSize/8);
    spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
    spiTransaction.args     = NULL;
    startTimeInUSec = ClockP_getTimeUsec();
    for(j = 0U; j < APP_MCSPI_TRANSFER_LOOPCOUNT; j++)
    {
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    }
    elapsedTimeInUsecs = ClockP_getTimeUsec() - startTimeInUSec;

    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("McSPI Clock %d Hz\r\n", gConfigMcspi0ChCfg[0U].bitRate);
    DebugP_log("----------------------------------------------------------\r\n");
    DebugP_log("Data Width \tData Length \tTransfer Time (micro sec)\r\n");
    DebugP_log("%u\t\t%u\t\t%5.2f\r\n", spiTransaction.dataSize, APP_MCSPI_MSGSIZE,
                        (Float32)(elapsedTimeInUsecs / (uint32_t)APP_MCSPI_TRANSFER_LOOPCOUNT));
    DebugP_log("----------------------------------------------------------\r\n\n");

    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        DebugP_assert(FALSE); /* MCSPI transfer failed!! */
    }
    else
    {

        /* Invalidate cache */
        CacheP_inv(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);
        /* Compare data */
        for(i = 0U; i < APP_MCSPI_MSGSIZE; i++)
        {
            if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
            {
                status = SystemP_FAILURE;   /* Data mismatch */
                DebugP_log("Data Mismatch at offset %d\r\n", i);
                break;
            }
        }
    }

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
