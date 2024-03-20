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

#include <drivers/qspi/v0/lld/qspi_lld.h>
#include <drivers/qspi/v0/lld/edma/qspi_edma_lld.h>
#include "qspi_nor_flash_1s_lld.h"

#include <string.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#define APP_QSPI_FLASH_OFFSET  (0x40000U)

#define APP_QSPI_DATA_SIZE (42*1024)


/* The source buffer used for transfer */
uint8_t gQspiTxBuf[APP_QSPI_DATA_SIZE];
/* Read buffer MUST be cache line aligned when using DMA */
uint8_t gQspiRxBuf[APP_QSPI_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void qspi_flash_diag_test_fill_buffers();
int32_t qspi_flash_diag_test_compare_buffers();
static __attribute__((target("arm"), aligned(4))) void App_EDMA_ISR(void);


void QSPIreadCompleteCallback (QSPILLD_Handle handle);

uint32_t transferMutex = MUTEX_ARM_LOCKED;

void qspi_flash_dma_lld(void *args)
{

    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint32_t manfId=0, deviceId=0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    (void) Board_driversOpen();

    gQspiHandle->readCompleteCallback =(QSPI_lld_dma_readCompleteCallback) &QSPIreadCompleteCallback;

    DebugP_log("[QSPI Flash Diagnostic Test] Starting ...\r\n");

    qspi_flash_diag_test_fill_buffers();

    (void) QSPI_norFlashInit(gQspiHandle);

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
        (void) QSPI_norFlashErase(gQspiHandle, offset);
        DebugP_log("[QSPI Flash Diagnostic Test] Done !!!\r\n");

        DebugP_log("[QSPI Flash Diagnostic Test] Performing Write-Read Test...\r\n");
        /* Performing Write to the flash */
        QSPI_norFlashWrite(gQspiHandle, offset, gQspiTxBuf, APP_QSPI_DATA_SIZE);
        /* Performing Read from the flash in DMA Mode*/
        QSPI_norFlashReadDma(gQspiHandle, offset, gQspiRxBuf, APP_QSPI_DATA_SIZE);
        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
        status |= qspi_flash_diag_test_compare_buffers();

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

    (void) QSPI_lld_deInit(gQspiHandle);
    EDMA_deinit();

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

int32_t qspi_flash_diag_test_compare_buffers()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_QSPI_DATA_SIZE; i++)
    {
        if(gQspiTxBuf[i] != gQspiRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("QSPI read data mismatch !!!\r\n");
            break;
        }
    }
    return status;
}

void QSPIreadCompleteCallback (QSPILLD_Handle handle)
{
    unlock_mutex(&transferMutex);
}
