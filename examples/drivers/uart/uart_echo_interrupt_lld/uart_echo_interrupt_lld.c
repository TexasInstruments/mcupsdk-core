/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 * When user types 'quit', the application ends.
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/uart/v0/lld/uart_lld.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_board_open_close.h"
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/* UART Driver handles */
UARTLLD_Handle gUartHandle0;

#define APP_UART_BUFSIZE              (200U)
#define APP_UART_RECEIVE_BUFSIZE      (8U)

uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

static __attribute__((target("arm"), aligned(4))) void App_UART_ISR(void);

uint32_t gMutexArmWrite = MUTEX_ARM_LOCKED;
uint32_t gMutexArmRead = MUTEX_ARM_LOCKED;

uint32_t gUartVimStsAddr;
uint32_t gUartVimStsClrMask;
uint32_t intcBaseAddr;
uint32_t intrNum;

void uart_echo_interrupt_lld(void *args)
{
    int32_t             transferOK;
    UART_Transaction    trans;

    Drivers_open();
    Board_driversOpen();\

    gUartHandle0 = &gUartObject[CONFIG_UART_CONSOLE];

    intrNum = gUartInitObject[CONFIG_UART_CONSOLE].intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gUartVimStsAddr = gHwiConfig.intcBaseAddr +  (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gUartVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    HwiP_setVecAddr(intrNum, (uintptr_t)&App_UART_ISR);
    HwiP_setPri(intrNum, gUartInitObject[CONFIG_UART_CONSOLE].intrPriority);
    HwiP_enableInt(intrNum);

    DebugP_log("[UART] Echo example started ...\r\n");

    gUartObject[CONFIG_UART_CONSOLE].writeTransferMutex = &gMutexArmWrite;
    gUartObject[CONFIG_UART_CONSOLE].readTransferMutex = &gMutexArmRead;

    UART_lld_Transaction_init(&trans);

    /* Send entry string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf,"This is uart echo test blocking mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_lld_writeIntr(gUartHandle0, trans.buf, trans.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART_CONSOLE].writeTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_lld_readIntr(gUartHandle0, trans.buf, trans.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART_CONSOLE].readTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Echo chars entered */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_lld_writeIntr(gUartHandle0, trans.buf, trans.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART_CONSOLE].writeTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Send exit string */
    gNumBytesWritten = 0U;
    trans.buf   = &gUartBuffer[0U];
    strncpy(trans.buf, "\r\nAll tests have passed!!\r\n", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_lld_writeIntr(gUartHandle0, trans.buf, trans.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART_CONSOLE].writeTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Deregister IRQ */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_setPri(intrNum, 15);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

static __attribute__((noinline, naked, target("arm"), aligned(4))) void App_UART_ISR(void)
{
   ISR_CALL_LEVEL_NONFLOAT_REENTRANT(UART_lld_controllerIsr, gUartHandle0, \
                                    intrNum, gUartVimStsAddr, \
                                    gUartVimStsClrMask, intcBaseAddr);
}

void UART_lld_writeCompleteCallback(void *args)
{
    unlock_mutex(gUartObject[CONFIG_UART_CONSOLE].writeTransferMutex);
    return;
}

void UART_lld_readCompleteCallback(void *args)
{
    unlock_mutex(gUartObject[CONFIG_UART_CONSOLE].readTransferMutex);
    return;
}