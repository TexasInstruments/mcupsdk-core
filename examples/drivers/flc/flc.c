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

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include <drivers/ospi.h>
#include <drivers/optiflash.h>
#include <kernel/dpl/DebugP.h>

#define APP_OSPI_FLASH_OFFSET_BASE  (0x200000U)

#define BUFFER_DATA_SIZE (4096)
uint8_t gTxBuff[BUFFER_DATA_SIZE];
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gRxBuf[BUFFER_DATA_SIZE] __attribute__((aligned(4096U))) = {0};

void fill_buffers(void);
int32_t compare_buffers(void);

void flc_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint32_t blk, page;

    /**
     * gFLCRegionConfig comes from syscfg generated files.
     * Here destination is being changed to gRxBuf.
     */
    gFLCRegionConfig[FLC_REGION_FLC0].destinationStartAddress = (uint32_t)(&(gRxBuf[0]));

    /* Open OSPI Driver, among others */
    Drivers_open();
    /* Open Flash drivers with OSPI instance as input */
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */

    offset = APP_OSPI_FLASH_OFFSET_BASE;
    fill_buffers();
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Block Erase Failed at 0x%X offset !!!", offset);
    }
    if(SystemP_SUCCESS == status)
    {
        status = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gTxBuff, BUFFER_DATA_SIZE);
    }
    else
    {
        DebugP_log("Flash Write of %d bytes failed at 0x%X offset !!!", BUFFER_DATA_SIZE, offset);
    }

    // enable DAC access

    OSPI_Handle ospihandle = OSPI_getHandle(CONFIG_OSPI0);
    OSPI_enableDacMode(ospihandle);

    if(SystemP_SUCCESS == status)
    {
        FLC_startRegion(&gFLCRegionConfig[0]);
        uint32_t cpy_status = 0;
        do
        {
            FLC_isRegionDone(&gFLCRegionConfig[FLC_REGION_FLC0], &cpy_status);
        } while ((cpy_status & (1<<FLC_REGION_FLC0)) == 0);
        status = SystemP_SUCCESS;
    }

    CacheP_inv(gRxBuf, BUFFER_DATA_SIZE, CacheP_TYPE_ALL);

    if(SystemP_SUCCESS == status)
    {
        status |= compare_buffers();
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

void fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < BUFFER_DATA_SIZE; i++)
    {
        gTxBuff[i] = i % 256;
    }
}

int32_t compare_buffers(void)
{
    int32_t status = SystemP_SUCCESS;
    volatile uint32_t i;

    for(i = 0U; i < BUFFER_DATA_SIZE; i++)
    {
        if(gTxBuff[i] != gRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("OSPI read data mismatch at %d!!!\r\n", i);
            break;
        }
    }
    return status;
}
