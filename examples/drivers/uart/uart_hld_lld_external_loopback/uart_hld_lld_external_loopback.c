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

/* This example demonstrates the UART RX and TX operation from both
 * hld and lld driver APIs by looping the Tx & Rx externally in polling
 * mode of operation.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_UART_BUFSIZE              (50U)

uint8_t gUartTxBuffer[APP_UART_BUFSIZE];
uint8_t gUartRxBuffer[APP_UART_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

void uart_hld_lld_external_loopback(void *args)
{
    int32_t          transferOK;
    UART_Transaction trans;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[UART] example started ...\r\n");
    DebugP_log("This is uart test with HLD-LLD instances\r\n");

    UART_Transaction_init(&trans);

    /* Send the characters from HLD driver API */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"Sending characters from HLD driver API", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);;
    transferOK = UART_write(gUartHandle[CONFIG_UART_HLD_INSTANCE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);


    /* Read the characters from lld driver API */
    gNumBytesRead = 0U;
    trans.buf   = &gUartRxBuffer[0U];
    transferOK = UART_lld_read(gUartHandleLld[CONFIG_UART_LLD_INSTANCE], trans.buf, trans.count, trans.timeout, NULL);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], strlen(trans.buf));
    DebugP_assert(transferOK == 0U);

    /* Send the characters from lld drvier API */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"Sending characters from LLD driver API\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_lld_write(gUartHandleLld[CONFIG_UART_LLD_INSTANCE], trans.buf, trans.count, trans.timeout, NULL);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read the characters from hld driver API */
    gNumBytesRead = 0U;
    trans.buf   = &gUartRxBuffer[0U];
    transferOK = UART_read(gUartHandle[CONFIG_UART_HLD_INSTANCE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], strlen(trans.buf));
    DebugP_assert(transferOK == 0U);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}