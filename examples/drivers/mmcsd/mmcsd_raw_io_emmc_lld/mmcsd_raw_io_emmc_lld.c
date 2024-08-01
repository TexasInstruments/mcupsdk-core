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
#include <kernel/dpl/ClockP.h>
#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_MMCSD_START_BLK         (0x300000U) /* @1.5GB */
#define APP_MMCSD_DATA_SIZE         (4096)

uint8_t gMmcsdTxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));
uint8_t gMmcsdRxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));

uint32_t writeTicks = 0U, readTicks = 0U;
uint32_t startTicks = 0;

void mmcsd_io_fill_buffers(void);

void mmcsd_raw_io_emmc_lld_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    int32_t     status = MMCSD_STS_SUCCESS;
    uint32_t    blockSize = MMCSD_lld_getBlockSize(gMmcsdLldHandle[0]);
    uint32_t    numBlocks = APP_MMCSD_DATA_SIZE / blockSize;

    if((APP_MMCSD_DATA_SIZE % blockSize) != 0)
    {
        numBlocks += 1;
    }

    DebugP_log("[MMCSD] eMMC Polling LLD Starting...\r\n");

    /* Fill Write and Read Buffer */
    mmcsd_io_fill_buffers();

    /* Initiate Transfer */
    status = MMCSD_lld_write_MMC_Poll(gMmcsdLldHandle[0], gMmcsdTxBuf,
                                      APP_MMCSD_START_BLK, numBlocks);

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Initiate Transfer */
        status = MMCSD_lld_read_MMC_Poll(gMmcsdLldHandle[0], gMmcsdRxBuf,
                                         APP_MMCSD_START_BLK, numBlocks);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        status = memcmp(gMmcsdRxBuf, gMmcsdTxBuf, APP_MMCSD_DATA_SIZE);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        DebugP_log("Data Matched !!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Data did not Match !!\r\n");
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void mmcsd_io_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < APP_MMCSD_DATA_SIZE; i++)
    {
        gMmcsdTxBuf[i] = i % 256;
        gMmcsdRxBuf[i] = 0U;
    }
}
