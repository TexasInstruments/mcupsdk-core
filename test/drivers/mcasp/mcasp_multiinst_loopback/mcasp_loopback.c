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

/* This example demonstrates the MibSPI RX and TX operation configured
 * in blocking, interrupt mode of operation.
 *
 * This example sends a known data in the TX mode of length
 * APP_MIBSPI_MSGSIZE and then receives the same in RX mode.
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

#define APP_MCASP_MSGSIZE       (1280U)
#define APP_MCASP_MSG_COUNT     (2U)
#define APP_MCASP_TEST_COUNT    (10U)

uint8_t gMcaspTxBuffer[CONFIG_MCASP_NUM_INSTANCES][APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE] __attribute__((aligned(256)));
uint8_t gMcaspRxBuffer[CONFIG_MCASP_NUM_INSTANCES][APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE] __attribute__((aligned(256)));
MCASP_Transaction   txnTx[CONFIG_MCASP_NUM_INSTANCES][APP_MCASP_MSG_COUNT] = {0};
MCASP_Transaction   txnRx[CONFIG_MCASP_NUM_INSTANCES][APP_MCASP_MSG_COUNT] = {0};
volatile uint32_t    gMcaspTestCntRx[CONFIG_MCASP_NUM_INSTANCES] = {0};
volatile uint32_t    gMcaspTestCntTx[CONFIG_MCASP_NUM_INSTANCES] = {0};

void mcasp_loopback_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i, j, k, l;
    MCASP_Handle mcaspHandle;
    volatile uint32_t   transferComplete = 0;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCASP] Multi instance Loopback example started.\r\n");

    /* Memfill buffers */
    for (i = CONFIG_MCASP0; i < CONFIG_MCASP_NUM_INSTANCES; i++)
    {
        uint32_t bufStartOffset = 64 * i;
        for (j = 0U; j < APP_MCASP_MSG_COUNT; j++)
        {
            for(k = 0U; k < APP_MCASP_MSGSIZE; k++)
            {
                gMcaspTxBuffer[i][j][k] = (bufStartOffset + k) % 256;
                gMcaspRxBuffer[i][j][k] = 0U;
            }
        }
    }

    for (i = CONFIG_MCASP0; i < CONFIG_MCASP_NUM_INSTANCES; i++)
    {
        mcaspHandle = MCASP_getHandle(i);

        for (j = 0U; j < APP_MCASP_MSG_COUNT; j++)
        {
            txnTx[i][j].buf = (void*) &gMcaspTxBuffer[i][j][0];
            txnTx[i][j].count = APP_MCASP_MSGSIZE/4;
            txnTx[i][j].timeout = 0xFFFFFF;
            txnTx[i][j].args = (void *) &(gMcaspTestCntTx[i]);
            MCASP_submitTx(mcaspHandle, &txnTx[i][j]);
        }

        for (j = 0U; j < APP_MCASP_MSG_COUNT; j++)
        {
            txnRx[i][j].buf = (void*) &gMcaspRxBuffer[i][j][0];
            txnRx[i][j].count = APP_MCASP_MSGSIZE/4;
            txnRx[i][j].timeout = 0xFFFFFF;
            txnRx[i][j].args = (void *) &(gMcaspTestCntRx[i]);
            MCASP_submitRx(mcaspHandle,  &txnRx[i][j]);
        }

        status = MCASP_startTransferRx(mcaspHandle);
        DebugP_assert(status == SystemP_SUCCESS);

        status = MCASP_startTransferTx(mcaspHandle);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    while (transferComplete == 0)
    {
        /* wait for transfer completion. */
        transferComplete = 1;
        for (i = CONFIG_MCASP0; i < CONFIG_MCASP_NUM_INSTANCES; i++)
        {
            if ((gMcaspTestCntRx[i] < APP_MCASP_TEST_COUNT -1 ) ||
                (gMcaspTestCntTx[i] < APP_MCASP_TEST_COUNT -1 ))
                {
                    transferComplete = 0;
                }
        }
    }

    /* withdraw the buffers submitted to driver. */
    if(SystemP_SUCCESS == status)
    {
        MCASP_Transaction *transaction;
        for (i = CONFIG_MCASP0; i < CONFIG_MCASP_NUM_INSTANCES; i++)
        {
            mcaspHandle = MCASP_getHandle(i);
            do {
                transaction = MCASP_withdrawRx(mcaspHandle);
            }while (transaction != NULL);
            do {
                transaction = MCASP_withdrawTx(mcaspHandle);
            }while (transaction != NULL);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        l = 0;
        /* Compare data */
        for (i = CONFIG_MCASP0; i < CONFIG_MCASP_NUM_INSTANCES; i++)
        {
            for (j = 0U; j < APP_MCASP_MSG_COUNT - 1; j++)
            {
                for(k = 0U; k < APP_MCASP_MSGSIZE; k++)
                {
                    if(gMcaspTxBuffer[i][j][k] != gMcaspRxBuffer[i][j][k])
                    {
                        status = SystemP_FAILURE;   /* Data mismatch */
                        l++;
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
        DebugP_log("Data mismatch for %d bytes!!\r\n", l);
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void mcasp_loopback_txcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    uint32_t *testCntTx = (uint32_t *)transaction->args;
    if (*testCntTx < APP_MCASP_TEST_COUNT - 1)
    {
        *testCntTx = *testCntTx + 1;
        MCASP_submitTx(handle, transaction);
    }
    else
    {
        MCASP_stopTransferTx(handle);
    }
}

void mcasp_loopback_rxcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    uint32_t *testCntRx = (uint32_t *)transaction->args;
    if (*testCntRx < APP_MCASP_TEST_COUNT - 1)
    {
        *testCntRx = *testCntRx + 1;
        MCASP_submitRx(handle, transaction);
    }
    else
    {
        MCASP_stopTransferRx(handle);
    }
}
