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

/* This example demonstrates the UART RX and TX operation by echoing char
 * that it recieves in blocking, interrupt mode of operation.
 * Two uart instances runs parallely asking the user to input 8 characters
 * and once both the transfer is completed successfully, the application ends.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_UART_BUFSIZE              (200U)
#define APP_UART_RECEIVE_BUFSIZE      (8U)

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Semaphore to indicate transfer completion */
SemaphoreP_Object gSemaphoreObj0;
SemaphoreP_Object gSemaphoreObj1;

static int32_t UART_echoDma(uint32_t uartConfig);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void UART_echoDmaTask0(void *args) 
{
    uint32_t uartConfig = (uint32_t)args;
    int32_t status = SystemP_SUCCESS;

    status = UART_echoDma(uartConfig);
    DebugP_assert(status==SystemP_SUCCESS);

    SemaphoreP_post(&gSemaphoreObj0);

    vTaskDelete(NULL);
}

void UART_echoDmaTask1(void *args) 
{
    uint32_t uartConfig = (uint32_t)args;
    int32_t status = SystemP_SUCCESS;

    status = UART_echoDma(uartConfig);
    DebugP_assert(status==SystemP_SUCCESS);

    SemaphoreP_post(&gSemaphoreObj1);

    vTaskDelete(NULL);
}

static int32_t UART_echoDma(uint32_t uartConfig)
{
    UART_Transaction trans;
    int32_t  transferOK = SystemP_SUCCESS;
    /* Application Buffers */
    uint8_t uartBuffer[APP_UART_BUFSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
    uint8_t uartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    UART_Transaction_init(&trans);

    /* Send entry string */
    trans.buf   = &uartBuffer[0U];
    strncpy(trans.buf,"This is uart echo test DMA blocking mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    CacheP_wb((void *)trans.buf, trans.count, CacheP_TYPE_ALL);
    transferOK = UART_write(gUartHandle[uartConfig], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    CacheP_wbInv((void *)&uartReceiveBuffer[0U], APP_UART_RECEIVE_BUFSIZE, CacheP_TYPE_ALL);
    /* Read 8 chars */
    trans.buf   = &uartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    CacheP_wbInv((void *)trans.buf, APP_UART_RECEIVE_BUFSIZE, CacheP_TYPE_ALL);
    transferOK = UART_read(gUartHandle[uartConfig], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Echo chars entered */
    trans.buf   = &uartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[uartConfig], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    return transferOK;
}

void UART_echoDma_multiInstance(void *args)
{
    int32_t status = SystemP_SUCCESS;
    UART_Transaction trans;
    /* Application Buffers */
    uint8_t uartBuffer[APP_UART_BUFSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    /* Create two tasks, one for each UART instance */
    TaskHandle_t uart0_task;
    TaskHandle_t uart1_task;

    Drivers_open();
    Board_driversOpen();

    SemaphoreP_constructBinary(&gSemaphoreObj0, 0);
    SemaphoreP_constructBinary(&gSemaphoreObj1, 0);

    xTaskCreate(UART_echoDmaTask0, "UART0", configMINIMAL_STACK_SIZE, (void *)CONFIG_UART0, tskIDLE_PRIORITY, &uart0_task);
    xTaskCreate(UART_echoDmaTask1, "UART1", configMINIMAL_STACK_SIZE, (void *)CONFIG_UART1, tskIDLE_PRIORITY, &uart1_task);

    /* Wait for the tasks to complete - this marks transfer completion */
    SemaphoreP_pend(&gSemaphoreObj0, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gSemaphoreObj1, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&gSemaphoreObj0);
    SemaphoreP_destruct(&gSemaphoreObj1);

    /* Send exit string */
    UART_Transaction_init(&trans);
    trans.buf   = &uartBuffer[0U];
    strncpy(trans.buf, "\r\nAll tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    CacheP_wb((void *)trans.buf, trans.count, CacheP_TYPE_ALL);
    status = UART_write(gUartHandle[CONFIG_UART0], &trans);
    APP_UART_ASSERT_ON_FAILURE(status, trans);

    Board_driversClose();
    Drivers_close();

    return;
}