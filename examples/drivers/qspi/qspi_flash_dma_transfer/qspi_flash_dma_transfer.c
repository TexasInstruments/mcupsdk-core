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

/**
 * This example performs QSPI transfer test using polling mode.
 *
 * The JEDEC ID of flash is read and printed out. A block
 * of flash memory is erased by sending erase command and
 * data is  written into the flash from a prefilled buffer.
 * Then, data is read back from it and validated.
 * If the equality test is successful, the test was successful.
 */

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#define APP_QSPI_FLASH_OFFSET  (0x40000U)

 #define APP_QSPI_DATA_SIZE (42*1024)

/* The source buffer used for transfer */
uint8_t gQspiTxBuf[APP_QSPI_DATA_SIZE];
/* Read buffer MUST be cache line aligned when using DMA */
uint8_t gQspiRxBuf[APP_QSPI_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void qspi_flash_diag_test_fill_buffers(void);
int32_t qspi_flash_diag_test_compare_buffers(void);
uint32_t transferMutex = MUTEX_ARM_LOCKED;

void qspi_flash_dma_transfer(void *args)
{

    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint32_t blk, page;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[QSPI Flash DMA Transfer Test] Starting ...\r\n");

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */
    offset = APP_QSPI_FLASH_OFFSET;

    qspi_flash_diag_test_fill_buffers();

    DebugP_log("[QSPI Flash DMA Transfer Test] Executing Flash Erase on first block...\r\n");
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    DebugP_log("[QSPI Flash DMA Transfer Test] Performing Write-Read Test...\r\n");
    Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gQspiTxBuf, APP_QSPI_DATA_SIZE);
    Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gQspiRxBuf, APP_QSPI_DATA_SIZE);
    status += qspi_flash_diag_test_compare_buffers();

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

void qspi_flash_diag_test_fill_buffers(void)
{
    uint32_t itr;

    for(itr = 0U; itr < APP_QSPI_DATA_SIZE; itr++)
    {
        gQspiTxBuf[itr] = itr;
        gQspiRxBuf[itr] = 0U;
    }
}

int32_t qspi_flash_diag_test_compare_buffers(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr;

    for(itr = 0U; itr < APP_QSPI_DATA_SIZE; itr++)
    {
        if(gQspiTxBuf[itr] != gQspiRxBuf[itr])
        {
            status = SystemP_FAILURE;
            break;
        }
    }
    return status;
}
