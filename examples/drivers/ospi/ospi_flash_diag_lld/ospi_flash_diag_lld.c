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

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <string.h>

/* Cypress flashes have hybrid sector configuration, this puts the first 256KB of the flash in
    hybrid sector mode. This will make block erases to first 256 KB fail. Not to lose generality,
    choosing the offset to be at 512 KB */
#define APP_OSPI_FLASH_OFFSET  (512*1024U)
#define APP_OSPI_DATA_SIZE (256)

uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE];
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gOspiRxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(128U)));

void ospi_flash_diag_test_fill_buffers(void);
int32_t ospi_flash_diag_test_compare_buffers(void);

void ospi_flash_diag_lld(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t manfId, deviceId;

    /* Open OSPI Driver, among others */
    Drivers_open();

    OSPI_lld_configResetPin(gOspiHandle,2);

    DebugP_log("[OSPI Flash Diagnostic Test] Starting ...\r\n");

#if defined (SOC_AM64X)
/* The OSPI Controller will be configured in 8s-8s-8s mode */
    OSPI_lld_setProtocol(gOspiHandle,525320);
#endif

    OSPI_lld_norFlashSetCmds(0x03, 0x02, 0xD8);

    /* Initialize the flash device in 1s1s1s mode */
    OSPI_lld_norFlashInit1s1s1s(gOspiHandle);

    /* Read ID */
    status = OSPI_lld_norFlashReadId(gOspiHandle, &manfId, &deviceId);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[OSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("[OSPI Flash Diagnostic Test] Flash Device ID       : 0x%X\r\n", deviceId);
    }

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */

    if( SystemP_SUCCESS == status)
    {
        ospi_flash_diag_test_fill_buffers();

        uint32_t offset  = APP_OSPI_FLASH_OFFSET;

        DebugP_log("[OSPI Flash Diagnostic Test] Executing Flash Erase on first block...\r\n");
        status = OSPI_lld_norFlashErase(gOspiHandle, offset);
        if(SystemP_SUCCESS == status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Done !!!\r\n");
        }
        else
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Erase Failed !!!\r\n");
        }
        DebugP_log("[OSPI Flash Diagnostic Test] Performing Write-Read Test...\r\n");
        status = OSPI_lld_norFlashWrite(gOspiHandle, offset, gOspiTxBuf, APP_OSPI_DATA_SIZE);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Wtite Failed !!!\r\n");
        }
        else
        {
            /* Nothing */
        }
        OSPI_lld_norFlashRead(gOspiHandle, offset, gOspiRxBuf, APP_OSPI_DATA_SIZE);

        status |= ospi_flash_diag_test_compare_buffers();

        if(SystemP_SUCCESS == status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Write-Read Test Passed!\r\n");
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

    Board_driversClose();
    Drivers_close();
}

void ospi_flash_diag_test_fill_buffers()
{
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        gOspiTxBuf[i] = i;
        gOspiRxBuf[i] = 0U;
    }
}

int32_t ospi_flash_diag_test_compare_buffers()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        if(gOspiTxBuf[i] != gOspiRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("OSPI read data mismatch !!!\r\n");
            break;
        }
    }
    return status;
}
