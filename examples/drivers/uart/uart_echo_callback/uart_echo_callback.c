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

/* This example demonstrates the UART RX and TX operation in callback,
 * interrupt mode of operation.
 * This example receives 8 characters and echos back the same.
 * The application ends when the user types 8 characters.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_UART_BUFSIZE              (200U)
#define APP_UART_RECEIVE_BUFSIZE      (8U)

uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

/* Semaphore to indicate Write/Read completion used in callback api's */
static SemaphoreP_Object gUartWriteDoneSem;
static SemaphoreP_Object gUartReadDoneSem;

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

void uart_echo_callback(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[UART] Echo callback example started ...\r\n");

    status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"This is uart echo test callback mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Wait for write completion */
    SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesWritten == strlen(trans.buf));

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Wait for read completion */
    SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesRead == APP_UART_RECEIVE_BUFSIZE);

    /* Echo chars entered */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Wait for write completion */
    SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesWritten == APP_UART_RECEIVE_BUFSIZE);

    /* Send exit string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf, "\r\nAll tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Wait for write completion */
    SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesWritten == strlen(trans.buf));

    SemaphoreP_destruct(&gUartWriteDoneSem);
    SemaphoreP_destruct(&gUartReadDoneSem);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

void uart_echo_write_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesWritten = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem);

    return;
}

void uart_echo_read_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesRead = trans->count;
    SemaphoreP_post(&gUartReadDoneSem);

    return;
}
