/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

#define APP_UART_CANCEL_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_CANCELLED != transaction.status)) \
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
static void uart_echo_read_partial_external_loopback_full_test(void *args);
static void uart_external_loopback_full_test(void *args);
static void uart_autobaud_external_loopback_full_test(void *args);
static void uart_write_cancel_external_loopback_full_test(void *args);
static void uart_read_cancel_external_loopback_full_test(void *args);
static void uart_six_bit_char_length_external_loopback_full_test(void *args);
#if defined(SOC_AM65X)
extern void UART_intr_router_configInit(void);
extern void UART_intr_router_configDeinit(void);
#endif

void test_main(void *args)
{
    UART_TestParams      testParams;

    Drivers_open();
    Board_driversOpen();
    #if defined(SOC_AM65X)
    UART_intr_router_configInit();
    #endif

    UNITY_BEGIN();

#if !defined(SOC_AM263X)
    test_uart_set_params(&testParams, 1111);
    RUN_TEST(uart_echo_read_full_test, 1111, (void*)&testParams);
    test_uart_set_params(&testParams, 1116);
    RUN_TEST(uart_echo_read_full_test, 1116, (void*)&testParams);
    test_uart_set_params(&testParams, 1117);
    RUN_TEST(uart_echo_read_partial_test, 1117, (void*)&testParams);
#else
    test_uart_set_params(&testParams, 12291);
    RUN_TEST(uart_external_loopback_full_test, 12291, (void*)&testParams);
    test_uart_set_params(&testParams, 12292);
    RUN_TEST(uart_external_loopback_full_test, 12292, (void*)&testParams);
    test_uart_set_params(&testParams, 12293);
    RUN_TEST(uart_echo_read_partial_external_loopback_full_test, 12293, (void*)&testParams);
    test_uart_set_params(&testParams, 12294);
    RUN_TEST(uart_external_loopback_full_test, 12294, (void*)&testParams);
    test_uart_set_params(&testParams, 12295);
    RUN_TEST(uart_external_loopback_full_test, 12295, (void*)&testParams);
    test_uart_set_params(&testParams, 12298);
    RUN_TEST(uart_six_bit_char_length_external_loopback_full_test, 12298, (void*)&testParams);
    test_uart_set_params(&testParams, 12299);
    RUN_TEST(uart_external_loopback_full_test, 12299, (void*)&testParams);
    test_uart_set_params(&testParams, 12300);
    RUN_TEST(uart_external_loopback_full_test, 12300, (void*)&testParams);
    test_uart_set_params(&testParams, 12301);
    RUN_TEST(uart_external_loopback_full_test, 12301, (void*)&testParams);
    test_uart_set_params(&testParams, 12302);
    RUN_TEST(uart_external_loopback_full_test, 12302, (void*)&testParams);
    test_uart_set_params(&testParams, 12304);
    RUN_TEST(uart_external_loopback_full_test, 12304, (void*)&testParams);
    test_uart_set_params(&testParams, 12314);
    RUN_TEST(uart_external_loopback_full_test, 12314, (void*)&testParams);
    test_uart_set_params(&testParams, 12315);
    RUN_TEST(uart_external_loopback_full_test, 12315, (void*)&testParams);
    test_uart_set_params(&testParams, 12316);
    RUN_TEST(uart_external_loopback_full_test, 12316, (void*)&testParams);
    test_uart_set_params(&testParams, 12317);
    RUN_TEST(uart_external_loopback_full_test, 12317, (void*)&testParams);
    test_uart_set_params(&testParams, 12318);
    RUN_TEST(uart_external_loopback_full_test, 12318, (void*)&testParams);
    test_uart_set_params(&testParams, 12319);
    RUN_TEST(uart_external_loopback_full_test, 12319, (void*)&testParams);
    test_uart_set_params(&testParams, 12320);
    RUN_TEST(uart_external_loopback_full_test, 12320, (void*)&testParams);
    test_uart_set_params(&testParams, 12321);
    RUN_TEST(uart_external_loopback_full_test, 12321, (void*)&testParams);
    test_uart_set_params(&testParams, 12322);
    RUN_TEST(uart_external_loopback_full_test, 12322, (void*)&testParams);
    test_uart_set_params(&testParams, 12323);
    RUN_TEST(uart_external_loopback_full_test, 12323, (void*)&testParams);
    test_uart_set_params(&testParams, 12324);
    RUN_TEST(uart_external_loopback_full_test, 12324, (void*)&testParams);
    test_uart_set_params(&testParams, 12325);
    RUN_TEST(uart_external_loopback_full_test, 12325, (void*)&testParams);
    test_uart_set_params(&testParams, 12326);
    RUN_TEST(uart_external_loopback_full_test, 12326, (void*)&testParams);
    test_uart_set_params(&testParams, 12327);
    RUN_TEST(uart_external_loopback_full_test, 12327, (void*)&testParams);
    test_uart_set_params(&testParams, 12305);
    RUN_TEST(uart_external_loopback_full_test, 12305, (void*)&testParams);
    test_uart_set_params(&testParams, 12306);
    RUN_TEST(uart_external_loopback_full_test, 12306, (void*)&testParams);
    test_uart_set_params(&testParams, 12328);
    RUN_TEST(uart_external_loopback_full_test, 12328, (void*)&testParams);
    test_uart_set_params(&testParams, 12329);
    RUN_TEST(uart_external_loopback_full_test, 12329, (void*)&testParams);
    test_uart_set_params(&testParams, 12330);
    RUN_TEST(uart_external_loopback_full_test, 12330, (void*)&testParams);
    test_uart_set_params(&testParams, 12331);
    RUN_TEST(uart_external_loopback_full_test, 12331, (void*)&testParams);
    test_uart_set_params(&testParams, 12307);
    RUN_TEST(uart_external_loopback_full_test, 12307, (void*)&testParams);
    test_uart_set_params(&testParams, 12308);
    RUN_TEST(uart_external_loopback_full_test, 12308, (void*)&testParams);
    test_uart_set_params(&testParams, 12309);
    RUN_TEST(uart_autobaud_external_loopback_full_test, 12309, (void*)&testParams);
    test_uart_set_params(&testParams, 12602);
    RUN_TEST(uart_write_cancel_external_loopback_full_test, 12602, (void*)&testParams);
    test_uart_set_params(&testParams, 12603);
    RUN_TEST(uart_read_cancel_external_loopback_full_test, 12603, (void*)&testParams);
#endif
    #if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM65X)
    test_uart_set_params(&testParams, 2514);
    RUN_TEST(uart_echo_read_full_test_dmaMode, 2514, (void*)&testParams);
    #endif

    /* Pass invalid test case ID to initialize with defaults */
    test_uart_set_params(&testParams, 0xFFFF);
    /* Redirect Pass logs to UART so that can be automated */
    test_printExitString((void*)&testParams);

    UNITY_END();

    #if defined(SOC_AM65X)
    UART_intr_router_configDeinit();
    #endif
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

#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM65X)
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

#if defined(SOC_AM263X)
static void uart_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

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
        DebugP_assert(strlen(trans.buf) == gNumBytesWritten);
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten == gNumBytesRead);
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], trans.count);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}

static void uart_six_bit_char_length_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"123456789", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(strlen(trans.buf) == gNumBytesWritten);
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten == gNumBytesRead);
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], trans.count);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}

static void uart_echo_read_partial_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1, strLen, tmpVar;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

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
        transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
        tmpVar = tmpVar - trans.count;
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], strLen);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}

static void uart_autobaud_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);

    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

    uartParams->operMode = UART_OPER_MODE_16X;
    uartParams->intrNum = CSLR_R5FSS0_CORE0_INTR_UART4_IRQ;
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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"AT : This is uart autobaud Test", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

     if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(strlen(trans.buf) == gNumBytesWritten);
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten == gNumBytesRead);
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], trans.count);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}

static void uart_write_cancel_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"This is uart write cancel Test"
                      "This is uart write cancel Test"
                      "This is uart write cancel Test...\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
    transferOK = UART_writeCancel(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_CANCEL_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_CANCEL_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
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

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}

static void uart_read_cancel_external_loopback_full_test(void *args)
{
    int32_t          transferOK, status;
    UART_Transaction trans;
    UART_Handle      uartHandle, uartHandle1;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_TestParams  *testParams1 = (UART_TestParams*)args;
    UART_Params     *uartParams = &(testParams->uartParams);
    UART_Params     *uartParams1 = &(testParams1->uartParams);
    uint32_t         baseAddr, baseAddr1;

    UART_close(gUartHandle[CONFIG_UART0]);
    UART_close(gUartHandle[CONFIG_UART1]);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    uartParams1->intrNum = CSLR_R5FSS0_CORE0_INTR_UART1_IRQ;
    uartHandle1 = UART_open(CONFIG_UART1, uartParams1);
    TEST_ASSERT_NOT_NULL(uartHandle1);

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
    baseAddr1 = UART_getBaseAddr(gUartHandle[CONFIG_UART1]);
    DebugP_assert(baseAddr1 != 0U);

    UART_disableLoopbackMode(baseAddr);
    UART_disableLoopbackMode(baseAddr1);

    UART_Transaction_init(&trans);

    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf,"This is uart read cancel Test"
                      "This is uart read cancel Test"
                      "This is uart read cancel Test...\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for write completion */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(strlen(trans.buf) == gNumBytesWritten);
    }

    trans.buf   = &gUartRxBuffer[0U];
    trans.count = trans.count;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
    transferOK = UART_readCancel(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_CANCEL_ASSERT_ON_FAILURE(transferOK, trans);

    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        /* Wait for read completion */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
    }

    transferOK = memcmp(&gUartTxBuffer[0U], &gUartRxBuffer[0U], gNumBytesRead);
    DebugP_assert(transferOK == 0U);

    if (uartParams->writeMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartWriteDoneSem);
    }
    if (uartParams->readMode == UART_TRANSFER_MODE_CALLBACK)
    {
        SemaphoreP_destruct(&gUartReadDoneSem);
    }

    UART_close(uartHandle);
    UART_close(uartHandle1);

    return;
}
#endif

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
#if !defined(SOC_AM65X)
    UART_Handle      uartHandle;
    UART_TestParams *testParams = (UART_TestParams*)args;
    UART_Params     *uartParams;

    uartParams = &(testParams->uartParams);
    uartHandle = UART_open(CONFIG_UART0, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);

#endif
    UART_Transaction_init(&trans);
    /* Send exit string */
    trans.buf   = &gUartTxBuffer[0U];
    strncpy(trans.buf, "All tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
#if defined(SOC_AM65X)
    CacheP_wb((void *)trans.buf, trans.count, CacheP_TYPE_ALL);
    transferOK = UART_write(gUartHandle[CONFIG_UART1], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
#else
    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    UART_close(uartHandle);
#endif

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

void uart_echo_writeCancel_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_CANCELLED == trans->status);
    gNumBytesWritten = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem);

    return;
}

void uart_echo_readCancel_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_CANCELLED == trans->status);
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
    params->intrNum = CSLR_R5FSS0_CORE0_INTR_UART4_IRQ;
    #endif
    #if defined(SOC_AM263PX)
    params->intrNum = CSLR_R5FSS0_CORE0_INTR_UART0_IRQ;
    #endif
    #if defined(SOC_AM62X)
    params->intrNum = CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART0_USART_IRQ_0 + 16;
    #endif
    #if defined(SOC_AM65X)
    params->intrNum = CSL_MCU0_INTR_MAIN2MCU_LVL_INTR0_OUTL_8;
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
        case 12291:
            params->transferMode = UART_CONFIG_MODE_POLLED;
            break;
        case 12292:
            params->readReturnMode = UART_READ_RETURN_MODE_FULL;
            break;
        case 12293:
            params->readReturnMode = UART_READ_RETURN_MODE_PARTIAL;
            break;
        case 12294:
            params->readMode = UART_TRANSFER_MODE_CALLBACK;
            params->writeMode = UART_TRANSFER_MODE_CALLBACK;
            params->readCallbackFxn = uart_echo_read_callback;
            params->writeCallbackFxn = uart_echo_write_callback;
            break;
        case 12295:
            params->readMode = UART_TRANSFER_MODE_BLOCKING;
            params->writeMode = UART_TRANSFER_MODE_BLOCKING;
            break;
        case 12298:
            params->dataLength =  UART_LEN_6;
            break;
        case 12299:
            params->dataLength =  UART_LEN_7;
            break;
        case 12300:
            params->parityType =  UART_PARITY_ODD;
            break;
        case 12301:
            params->parityType =  UART_PARITY_EVEN;
            break;
        case 12302:
            params->stopBits =  UART_STOPBITS_2;
            break;
        case 12304:
            params->baudRate =  300;
            break;
        case 12314:
            params->baudRate =  600;
            break;
        case 12315:
            params->baudRate =  1200;
            break;
        case 12316:
            params->baudRate =  2400;
            break;
        case 12317:
            params->baudRate =  4800;
            break;
        case 12318:
            params->baudRate =  9600;
            break;
        case 12319:
            params->baudRate =  14400;
            break;
        case 12320:
            params->baudRate =  19200;
            break;
        case 12321:
            params->baudRate =  38400;
            break;
        case 12322:
            params->baudRate =  57600;
            break;
        case 12323:
            params->baudRate =  230400;
            break;
        case 12324:
            params->operMode = UART_OPER_MODE_13X;
            params->baudRate =  460800;
            break;
        case 12325:
            params->operMode = UART_OPER_MODE_13X;
            params->baudRate =  921600;
            break;
        case 12326:
            params->operMode = UART_OPER_MODE_13X;
            params->baudRate =  1843000;
            break;
        case 12327:
            params->operMode = UART_OPER_MODE_13X;
            params->baudRate =  3688400;
            break;
        case 12305:
            params->timeGuardVal =  10;
            break;
        case 12306:
            params->rxTrigLvl = 1;
            params->txTrigLvl = 1;
            break;
        case 12328:
            params->rxTrigLvl = 8;
            params->txTrigLvl = 8;
            break;
        case 12329:
            params->rxTrigLvl = 16;
            params->txTrigLvl = 16;
            break;
        case 12330:
            params->rxTrigLvl = 56;
            params->txTrigLvl = 32;
            break;
        case 12331:
            params->rxTrigLvl = 60;
            params->txTrigLvl = 56;
            break;
        case 12307:
            params->hwFlowControl = (uint32_t)TRUE;
            params->hwFlowControlThr = UART_RXTRIGLVL_60;
            break;
        case 12308:
            params->operMode = UART_OPER_MODE_13X;
            params->baudRate = 921600;
            break;
        case 12309:
            params->operMode = UART_OPER_MODE_16X_AUTO_BAUD;
            params->baudRate = 57600;
            break;
        case 12602:
            params->readMode = UART_TRANSFER_MODE_CALLBACK;
            params->writeMode = UART_TRANSFER_MODE_CALLBACK;
            params->readCallbackFxn = uart_echo_read_callback;
            params->writeCallbackFxn = uart_echo_writeCancel_callback;
            break;
        case 12603:
            params->readMode = UART_TRANSFER_MODE_CALLBACK;
            params->writeMode = UART_TRANSFER_MODE_CALLBACK;
            params->readCallbackFxn = uart_echo_readCancel_callback;
            params->writeCallbackFxn = uart_echo_write_callback;
            break;
    }

    return;
}