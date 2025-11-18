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

/* This example demonstrates the MibSPI RX and TX operation configured
 * in DMA mode of operation. This example configure different auxclk 
 * source and divider for MCASP module.
 *
 * This example sends a known data in the TX mode of length
 * APP_MCASP_MSGSIZE  and then receives the same in RX mode.
 * Digital loopback mode is enabled to receive data.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "drivers/mcasp.h"
#include <string.h>

#define APP_MCASP_MSGSIZE       (1280U)
#define APP_MCASP_MSG_COUNT     (5U)
#define APP_MCASP_TEST_COUNT    (10U)

uint8_t gMcaspTxBuffer[APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE] __attribute__((aligned(256)));
uint8_t gMcaspRxBuffer[APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE] __attribute__((aligned(256)));
MCASP_Transaction   txnTx[APP_MCASP_MSG_COUNT] = {0};
MCASP_Transaction   txnRx[APP_MCASP_MSG_COUNT] = {0};
volatile uint32_t    gMcaspTestCntRx = 0;
volatile uint32_t    gMcaspTestCntTx = 0;

extern uint8_t gTxLoopjobBuf0[];
extern uint8_t gRxLoopjobBuf0[];
extern uint8_t gTxLoopjobBuf1[];
extern uint8_t gRxLoopjobBuf1[];
extern uint8_t gTxLoopjobBuf2[];
extern uint8_t gRxLoopjobBuf2[];

void mcasp_auxclk_sel_main(void *args)
{
    int32_t      status = SystemP_SUCCESS;
    uint32_t     i=0, j=0, k=0, instIdx = 0U;
    MCASP_Handle mcaspHandle;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCASP] Loopback example started. Testing %d bytes ...\r\n",
                                    (APP_MCASP_MSG_COUNT * APP_MCASP_MSGSIZE));

    /* Memfill buffers */
    for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
    {
        for(j = 0U; j < APP_MCASP_MSGSIZE; j++)
        {
            gMcaspTxBuffer[i][j] = j % 256;
            gMcaspRxBuffer[i][j] = 0U;
        }
    }

    CacheP_wb(gMcaspTxBuffer, APP_MCASP_MSG_COUNT * APP_MCASP_MSGSIZE, CacheP_TYPE_ALL);
    CacheP_wb(gMcaspRxBuffer, APP_MCASP_MSG_COUNT * APP_MCASP_MSGSIZE, CacheP_TYPE_ALL);

    for (i = 0; i < 256; i++)
    {
        gTxLoopjobBuf0[i] = 0xa5;
        gRxLoopjobBuf0[i] = 0;
        gTxLoopjobBuf1[i] = 0xa5;
        gRxLoopjobBuf1[i] = 0;
        gTxLoopjobBuf2[i] = 0xa5;
        gRxLoopjobBuf2[i] = 0;
    }

    CacheP_wb(gTxLoopjobBuf0, 256, CacheP_TYPE_ALL);
    CacheP_wb(gRxLoopjobBuf0, 256, CacheP_TYPE_ALL);
    CacheP_wb(gTxLoopjobBuf1, 256, CacheP_TYPE_ALL);
    CacheP_wb(gRxLoopjobBuf1, 256, CacheP_TYPE_ALL);
    CacheP_wb(gTxLoopjobBuf2, 256, CacheP_TYPE_ALL);
    CacheP_wb(gRxLoopjobBuf2, 256, CacheP_TYPE_ALL);

    for (instIdx = 0U; instIdx < CONFIG_MCASP_NUM_INSTANCES; instIdx++)
    {
        mcaspHandle = MCASP_getHandle(instIdx);

        for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
        {
            txnTx[i].buf = (void*) &gMcaspTxBuffer[i][0];
            txnTx[i].count = APP_MCASP_MSGSIZE/4;
            txnTx[i].timeout = 0xFFFFFF;
            MCASP_submitTx(mcaspHandle, &txnTx[i]);
        }

        for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
        {
            txnRx[i].buf = (void*) &gMcaspRxBuffer[i][0];
            txnRx[i].count = APP_MCASP_MSGSIZE/4;
            txnRx[i].timeout = 0xFFFFFF;
            MCASP_submitRx(mcaspHandle,  &txnRx[i]);
        }

        status = MCASP_startTransferRx(mcaspHandle);
        DebugP_assert(status == SystemP_SUCCESS);

        status = MCASP_startTransferTx(mcaspHandle);
        DebugP_assert(status == SystemP_SUCCESS);

        while ((gMcaspTestCntRx < APP_MCASP_TEST_COUNT ) ||
            (gMcaspTestCntTx < APP_MCASP_TEST_COUNT ))
        {
            /* wait for transfer completion. */
        }

        MCASP_stopTransferRx(mcaspHandle);
        MCASP_stopTransferTx(mcaspHandle);

        /* withdraw the buffers submitted to driver. */
        if(SystemP_SUCCESS == status)
        {
            MCASP_Transaction *transaction;
            do {
                transaction = MCASP_withdrawRx(mcaspHandle);
            }while (transaction != NULL);
            do {
                transaction = MCASP_withdrawTx(mcaspHandle);
            }while (transaction != NULL);
        }

        CacheP_inv(gMcaspRxBuffer, APP_MCASP_MSGSIZE * APP_MCASP_MSG_COUNT, CacheP_TYPE_ALL);

        if(SystemP_SUCCESS == status)
        {
            k = 0;
            /* Compare data */
            for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
            {
                for(j = 0U; j < APP_MCASP_MSGSIZE; j++)
                {
                    if(gMcaspTxBuffer[i][j] != gMcaspRxBuffer[i][j])
                    {
                        status = SystemP_FAILURE;   /* Data mismatch */
                        k++;
                    }
                }
            }
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All bytes match!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Data mismatch for %d bytes!!\r\n", k);
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void mcasp_loopback_txcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntTx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntTx++;
        MCASP_submitTx(handle, transaction);
    }
}

void mcasp_loopback_rxcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntRx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntRx++;
        MCASP_submitRx(handle, transaction);
    }
}

void mcasp_loopback_txcb1 (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntTx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntTx++;
        MCASP_submitTx(handle, transaction);
    }
}

void mcasp_loopback_rxcb1 (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntRx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntRx++;
        MCASP_submitRx(handle, transaction);
    }
}

void mcasp_loopback_txcb2 (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntTx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntTx++;
        MCASP_submitTx(handle, transaction);
    }
}

void mcasp_loopback_rxcb2 (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    if (gMcaspTestCntRx < APP_MCASP_TEST_COUNT)
    {
        gMcaspTestCntRx++;
        MCASP_submitRx(handle, transaction);
    }
}
