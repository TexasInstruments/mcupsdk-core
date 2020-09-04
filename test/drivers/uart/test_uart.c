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

/* This example demonstrates the UART RX and TX operation in blocking mode
 * and in particular UART Read Return Partial Mode.
 * UART_READ_RETURN_MODE_PARTIAL unblocks or performs a callback whenever a
 * read timeout error occurs on the UART peripheral.
 * The read timeout occurs if the read FIFO is non-empty and no new
 * data has been received for a specific device/baudrate dependent number of
 * clock cycles.  This mode can be used when the exact number of bytes to
 * be read is not known.
 * Example is configured to receive APP_UART_RECEIVE_BUFSIZE characters but
 * the input to this test is a file which contains half of the APP_UART_RECEIVE_BUFSIZE
 * characters.
 * Example ends when it receives half of the APP_UART_RECEIVE_BUFSIZE characters.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_UART_BUFSIZE              (2048U)

uint8_t gUartTxBuffer[APP_UART_BUFSIZE];
uint8_t gUartRxBuffer[APP_UART_BUFSIZE];
uint32_t gNumBytesWritten = 0U, gNumBytesRead = 0U;

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

typedef struct UART_TestParams_s {

    UART_Params  uartParams;
} UART_TestParams;

static void test_uart_set_params(UART_TestParams *testParams, uint32_t testCaseId);
static void uart_echo_read_full_test(void *args);
static void uart_echo_read_partial_test(void *args);
static void test_printExitString(void *args);
static void uart_echo_read_full_test_dmaMode(void *args);

void test_main(void *args)
{
    UART_TestParams      testParams;

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_uart_set_params(&testParams, 1111);
    RUN_TEST(uart_echo_read_full_test, 1111, (void*)&testParams);
    test_uart_set_params(&testParams, 1116);
    RUN_TEST(uart_echo_read_full_test, 1116, (void*)&testParams);
    test_uart_set_params(&testParams, 1117);
    RUN_TEST(uart_echo_read_partial_test, 1117, (void*)&testParams);
    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    test_uart_set_params(&testParams, 2514);
    RUN_TEST(uart_echo_read_full_test_dmaMode, 2514, (void*)&testParams);
    #endif

    /* Pass invalid test case ID to initialize with defaults */
    test_uart_set_params(&testParams, 0xFFFF);
    /* Redirect Pass logs to UART so that can be automated */
    test_printExitString((void*)&testParams);

    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

static void uart_echo_read_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    uint32_t         baseAddr;

    UART_close(gUartHandle[CONFIG_UART0]);

    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    baseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART0]);
    DebugP_assert(baseAddr != 0U);

    UART_enableLoopbackMode(baseAddr);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"This is uart echo read FULL mode Test...\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(trans.count == strlen(trans.buf));
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(trans.count == strlen(trans.buf));
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], strlen(trans.buf));
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_disableLoopbackMode(baseAddr);
    UART_close(uartHandle);

    return;
}

#if !defined(SOC_AM62X)
static void uart_echo_read_full_test_dmaMode(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    uint32_t         baseAddr;

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    baseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr != 0U);

    UART_enableLoopbackMode(baseAddr);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"This is uart echo read FULL DMA mode Callback mode Test...\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    CacheP_wb((void *)trans.buf, trans.count, CacheP_TYPE_ALL);
    transferOK = UART_write(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(strlen(trans.buf) == gNumBytesWritten);
    }

    CacheP_wbInv((void *)&gUartRxBuffer[0U], APP_UART_BUFSIZE, CacheP_TYPE_ALL);

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = gNumBytesWritten;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten == gNumBytesRead);
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], gNumBytesWritten);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_disableLoopbackMode(baseAddr);

    return;
}
#endif

static void uart_echo_read_partial_test(void *args)
{
    int32_t          transferOK;
    UART_Transaction trans;
    UART_Handle      uartHandle;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    uint32_t         baseAddr, strLen, tmpVar;

    UART_close(gUartHandle[CONFIG_UART0]);

    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);

    baseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART0]);
    DebugP_assert(baseAddr != 0U);

    UART_enableLoopbackMode(baseAddr);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"This is uart echo read Partial mode Test...\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    strLen      = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = strLen;
    tmpVar      = strLen;
    while(tmpVar != 0U)
    {
        trans.buf   = &gUartRxBuffer[strLen - tmpVar];
        trans.count = tmpVar;
        transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);
        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
        tmpVar = tmpVar - trans.count;
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], strLen);
    DebugP_assert(transferOK == 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_close(uartHandle);

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

static void test_printExitString(void *args)
{
    int32_t          transferOK;
    UART_Transaction trans;
    UART_Handle      uartHandle;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_Params     *uartParams;

    uartParams = &(testParams->uartParams);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);

    UART_Transaction_init(&trans);
    /* Send exit string */
    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf, "All tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    UART_close(uartHandle);

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

static void test_uart_set_params(UART_TestParams *testParams, uint32_t tcId)
{
    UART_Params *params = &(testParams->uartParams);

    UART_Params_init(params);
    params->readReturnMode = UART_READ_RETURN_MODE_FULL;

    /* Map to interrupt line for UART in the SOC */
    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    params->intrNum = CSLR_R5FSS0_CORE0_INTR_UART0_USART_IRQ_0;
    #endif
    #if defined(SOC_AM263X)
    params->intrNum = CSLR_R5FSS0_CORE0_INTR_UART0_IRQ;
    #endif
    #if defined(SOC_AM62X)
    params->intrNum = CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART0_USART_IRQ_0 + 16;
    #endif
    switch (tcId)
    {
        case 1111:
            params->transferMode = UART_CONFIG_MODE_POLLED;
            break;
        case 1116:
            params->readReturnMode = UART_READ_RETURN_MODE_FULL;
            break;
        case 1117:
            params->readReturnMode = UART_READ_RETURN_MODE_PARTIAL;
            break;
        case 2514:
            params->readMode = UART_TRANSFER_MODE_CALLBACK;
            params->writeMode = UART_TRANSFER_MODE_CALLBACK;
            break;
    }

    return;
}
