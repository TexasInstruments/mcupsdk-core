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
 *  \file  test_uart_v1.c
 *
 * \brief This file contains demonstrates the UART RX and TX operation by echoing char
 * that it recieves in blocking interrupt mode of operation
 * the UART RX and TX operation in blocking mode.
 * In read write cancel test clear the the stored BUFFER and wait for next input.
 * In the interrupt read write test case making the interrupt  disable.
 * The read timeout occurs if the read FIFO is non-empty and no new
 * data has been received for a specific device/baudrate dependent number of
 * clock cycles.  This mode can be used when the exact number of bytes to
 * be read is not known.
 * 
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define UART_TEST_TIMEOUT           (10000U)
#define APP_UART_BUFSIZE            (2048U)
#define CONFIG_BLOKING_MODE           0x01
#define CONFIG_CALLBACK_MODE          0x02
#define APP_UART_RECEIVE_BUFSIZE      (8U)
#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \
	
	
uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

/* Semaphore to indicate Write/Read completion used in callback api's */
static SemaphoreP_Object gUartWriteDoneSem;
static SemaphoreP_Object gUartReadDoneSem;

static void test_uart_set_params(UART_Params *testParams, uint32_t tcId);
static void block_mode_read_write_test(void *args);
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
static void uartB_write_test(void *args);
#endif
static void uart_echo_callback_test(void *args);
static void uart_read_write_cancel_test(void *args);
static void uart_int_disable_read_write_test(void *args);
static void uart_timeout_test(void *args);
static void uart_echo_write_callback(UART_Handle handle, UART_Transaction *trans);
static void uart_echo_read_callback(UART_Handle handle, UART_Transaction *trans);
static void test_printExitString(void *args);

void test_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();
    
	test_uart_set_params(&gUartParams[CONFIG_UART_CONSOLE], CONFIG_BLOKING_MODE);
	RUN_TEST(block_mode_read_write_test, 2103, (void*)&gUartParams[CONFIG_UART_CONSOLE]);
	test_uart_set_params(&gUartParams[CONFIG_UART_CONSOLE], CONFIG_CALLBACK_MODE);
	RUN_TEST(uart_echo_callback_test, 2104, (void*)&gUartParams[CONFIG_UART_CONSOLE]);
    RUN_TEST(uart_read_write_cancel_test, 2105, (void*)&gUartParams[CONFIG_UART_CONSOLE]);
	test_uart_set_params(&gUartParams[CONFIG_UART_CONSOLE], CONFIG_BLOKING_MODE);
	RUN_TEST(uart_int_disable_read_write_test, 2106, (void*)&gUartParams[CONFIG_UART_CONSOLE]);
	RUN_TEST(uart_timeout_test, 2107, (void*)&gUartParams[CONFIG_UART_CONSOLE]);
	/* Redirect Pass logs to UART so that can be automated */
	test_printExitString((void*)&gUartParams[CONFIG_UART_CONSOLE]);
 
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    test_uart_set_params(&gUartParams[CONFIG_UARTB_CONSOLE], CONFIG_BLOKING_MODE);
    RUN_TEST(uartB_write_test, 2163, (void*)&gUartParams[CONFIG_UARTB_CONSOLE]);
#endif
    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

static void block_mode_read_write_test(void *args)
{
	int32_t         transferOK;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;  
	
	UART_close(gUartHandle[CONFIG_UART_CONSOLE]);
	
	uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
	
    UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rThis is uart echo test blocking mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Echo chars entered */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
	
	UART_close(uartHandle);
	
    return;
}

#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
static void uartB_write_test(void *args)
{
	int32_t         transferOK;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;  
	
	UART_close(gUartHandle[CONFIG_UARTB_CONSOLE]);
	
	uartHandle = UART_open(CONFIG_UARTB_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
	
    UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rUARTB Test Passed!!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UARTB_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_read(gUartHandle[CONFIG_UARTB_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

	UART_close(uartHandle);
	
    return;
}
#endif

static void uart_echo_callback_test(void *args)
{
	int32_t         transferOK,status;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;
	
	UART_close(gUartHandle[CONFIG_UART_CONSOLE]);
	
    status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
	
	uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    
	UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rThis is uart echo test callback mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
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
	
	UART_close(uartHandle);
	
    return;
}

static void uart_read_write_cancel_test(void *args)
{
	int32_t         transferOK,status;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;
	
	UART_close(gUartHandle[CONFIG_UART_CONSOLE]);
	
    status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
	
	uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
    
	UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rThis is read write cancel test, enter less than 8 char. Please input..\r\n", APP_UART_BUFSIZE);
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
    UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
	ClockP_usleep(100);
	UART_readCancel(uartHandle,&trans);
    /* Wait for read completion */
    SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesRead == APP_UART_RECEIVE_BUFSIZE);

	 /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\r\nPrevious read canceled,Receives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
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

	UART_Transaction_init(&trans);
	gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    trans.count = strlen(trans.buf);
	strncpy(trans.buf,"\n\rThis is write cancel test\r\n", APP_UART_BUFSIZE);
    UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
	UART_writeCancel(uartHandle,&trans);
	
	/* Wait for write completion */
    SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(gNumBytesWritten == trans.count);
	
	UART_close(uartHandle);
	
    return;
}

static void uart_int_disable_read_write_test(void *args)
{
	int32_t         transferOK;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;
	
	UART_close(gUartHandle[CONFIG_UART_CONSOLE]);
	uartParams->transferMode=UART_CONFIG_MODE_POLLED;

	uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
	
    UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rThis is uart interrupt disable read write test \r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
	
    /* Echo chars entered */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);
	
	UART_close(uartHandle);
	
    return;
}

static void uart_timeout_test(void *args)
{
	int32_t         transferOK;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;
	
	UART_close(gUartHandle[CONFIG_UART_CONSOLE]);
	
	uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle);
	
    UART_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"\n\rThis is uart timeout test\r\nwait for 10 seconds to timeout read\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
	trans.timeout  = UART_TEST_TIMEOUT;
    transferOK=UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
	DebugP_assert( UART_TRANSFER_STATUS_TIMEOUT == trans.status);
	
	UART_close(uartHandle);
    
	return;	
}

static void uart_echo_write_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesWritten = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem);
	
    return;
}

static void uart_echo_read_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesRead = trans->count;
    SemaphoreP_post(&gUartReadDoneSem);

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
	int32_t         transferOK;
    UART_Transaction trans;
	UART_Handle      uartHandle;
    UART_Params *	uartParams = (UART_Params*)args;

    uartHandle = UART_open(CONFIG_UART_CONSOLE, uartParams);
    TEST_ASSERT_NOT_NULL(uartHandle); 
	
    UART_Transaction_init(&trans);
    /* Send exit string */
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf, "\n\rAll tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    UART_close(uartHandle);

    return;
}

static void test_uart_set_params(UART_Params *testParams, uint32_t tcId)
{
    UART_Params *params = (testParams);

    switch (tcId)
    {
        case CONFIG_BLOKING_MODE:
			params->readMode           = UART_TRANSFER_MODE_BLOCKING;
			params->writeMode          = UART_TRANSFER_MODE_BLOCKING;
			params->readCallbackFxn    = NULL;
			params->writeCallbackFxn   = NULL;	
        break;
		case CONFIG_CALLBACK_MODE:
			params->readMode           = UART_TRANSFER_MODE_CALLBACK;
			params->writeMode          = UART_TRANSFER_MODE_CALLBACK;
			params->readCallbackFxn    = uart_echo_read_callback;
			params->writeCallbackFxn   = uart_echo_write_callback;		
        break;

    }

    return;
}
