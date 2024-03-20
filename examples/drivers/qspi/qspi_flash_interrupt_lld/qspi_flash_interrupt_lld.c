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

/**
 * This example performs QSPI transfer test using polling mode.
 *
 * The JEDEC ID of flash is read and printed out. A block
 * of flash memory is erased by sending erase command and
 * data is  written into the flash from a prefilled buffer.
 * Then, data is read back from it and validated.
 * If the equality test is successful, the test was successful.
 */

#include "qspi_nor_flash_1s_lld.h"
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <string.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#define APP_QSPI_FLASH_OFFSET  (0x40000U)

#define APP_QSPI_DATA_SIZE (256)

uint32_t intrNum;
uint32_t gQSPIVimStsAddr, intrNum, gQSPIVimStsClrMask, intcBaseAddr;
QSPILLD_WriteCmdParams msg = {0};
uint32_t offset;

/* The source buffer used for transfer */
uint8_t gQspiTxBuf[APP_QSPI_DATA_SIZE];
/* Read buffer MUST be cache line aligned when using DMA */
uint8_t gQspiRxBuf[APP_QSPI_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void qspi_flash_diag_test_fill_buffers();
int32_t qspi_flash_diag_test_compare_buffers();

void isrCallback(void *args);
uint32_t transferMutex = MUTEX_ARM_LOCKED;
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_QSPI_ISR(void);

void qspi_flash_interrupt_lld(void *args)
{

    int32_t status = SystemP_SUCCESS;

    uint32_t manfId=0, deviceId=0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    gQspiHandle->interruptCallback = isrCallback;
    intrNum = gQspiHandle->hQspiInit->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gQSPIVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gQSPIVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

        /* Register Interrupt */
    HwiP_setPri(intrNum, 4U);
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_QSPI_ISR);
    HwiP_enableInt(intrNum);
    HwiP_setAsPulse(intrNum, TRUE);

    DebugP_log("[QSPI Flash Diagnostic Test] Starting ...\r\n");

    qspi_flash_diag_test_fill_buffers();

    QSPI_norFlashInit(gQspiHandle);

    /* Read ID */
    status = QSPI_norFlashReadId(gQspiHandle, &manfId, &deviceId);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[QSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("[QSPI Flash Diagnostic Test] Flash Device ID       : 0x%X\r\n", deviceId);
    }

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */
    offset = APP_QSPI_FLASH_OFFSET;

    if( SystemP_SUCCESS == status)
    {
        qspi_flash_diag_test_fill_buffers();

        DebugP_log("[QSPI Flash Diagnostic Test] Executing Flash Erase on first block...\r\n");
        status = QSPI_norFlashErase(gQspiHandle, offset);
        DebugP_log("[QSPI Flash Diagnostic Test] Done !!!\r\n");
        DebugP_log("[QSPI Flash Diagnostic Test] Performing Write-Read Test...\r\n");

        /* Populating the command and Tx buffer*/
        msg.dataLen = APP_QSPI_DATA_SIZE;
        msg.dataBuf = gQspiTxBuf;
        msg.cmdAddr = offset;

        /* QSPI Interrupt Write */
        QSPI_norFlashWriteIntr(gQspiHandle, &msg);

        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
        QSPI_norFlashWaitReady(gQspiHandle, 400U);

        transferMutex = MUTEX_ARM_LOCKED;
        /* Populating the command and Rx buffer */
        msg.dataBuf = gQspiRxBuf;
        msg.cmd = 0x03;
        msg.numAddrBytes = 3U;
        /* QSPI Interrupt Read */
        QSPI_lld_readCmdIntr(gQspiHandle,&msg);

        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
        status += qspi_flash_diag_test_compare_buffers();

        if(SystemP_SUCCESS == status)
        {
            DebugP_log("[QSPI Flash Diagnostic Test] Write-Read Test Passed!\r\n");
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

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);

    Board_driversClose();
    Drivers_close();
}

void qspi_flash_diag_test_fill_buffers()
{
    uint32_t i;

    for(i = 0U; i < APP_QSPI_DATA_SIZE; i++)
    {
        gQspiTxBuf[i] = i;
        gQspiRxBuf[i] = 0U;
    }
}

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_QSPI_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(QSPI_lld_isr, \
                                      gQspiHandle, \
                                      intrNum, \
                                      gQSPIVimStsAddr, \
                                      gQSPIVimStsClrMask,
                                      intcBaseAddr);
}


void isrCallback(void *args)
{
    unlock_mutex(&transferMutex);
}

int32_t qspi_flash_diag_test_compare_buffers()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_QSPI_DATA_SIZE; i++)
    {
        if(gQspiTxBuf[i] != gQspiRxBuf[i])
        {
            status = SystemP_FAILURE;
            break;
        }
    }
    return status;
}
