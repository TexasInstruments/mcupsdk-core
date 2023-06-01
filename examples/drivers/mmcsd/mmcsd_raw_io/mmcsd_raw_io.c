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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_MMCSD_START_BLK (0x300000U) /* @1.5GB */
#define APP_MMCSD_DATA_SIZE (1024)

uint8_t gMmcsdTxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));
uint8_t gMmcsdRxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));

void mmcsd_raw_io_fill_buffers(void);

void mmcsd_raw_io_main(void *args)
{
    int32_t status = SystemP_SUCCESS;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    uint32_t blockSize = MMCSD_getBlockSize(gMmcsdHandle[CONFIG_MMCSD0]);
    uint32_t numBlocks = APP_MMCSD_DATA_SIZE / blockSize;

    if((APP_MMCSD_DATA_SIZE % blockSize) != 0)
    {
        numBlocks += 1;
    }

    DebugP_log("[MMCSD RAW IO] Starting...\r\n");

    mmcsd_raw_io_fill_buffers();

    /* Write known data */
    status = MMCSD_write(gMmcsdHandle[CONFIG_MMCSD0], gMmcsdTxBuf, APP_MMCSD_START_BLK, numBlocks);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Read back written data */
    status = MMCSD_read(gMmcsdHandle[CONFIG_MMCSD0], gMmcsdRxBuf, APP_MMCSD_START_BLK, numBlocks);
    DebugP_assert(status == SystemP_SUCCESS);

    status = memcmp(gMmcsdRxBuf, gMmcsdTxBuf, APP_MMCSD_DATA_SIZE);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    MMCSD_close(gMmcsdHandle[CONFIG_MMCSD0]);
    gMmcsdHandle[CONFIG_MMCSD0] = NULL;

    MMCSD_deinit();

    Board_driversClose();
    Drivers_close();
}

void mmcsd_raw_io_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < APP_MMCSD_DATA_SIZE; i++)
    {
        gMmcsdTxBuf[i] = i % 256;
        gMmcsdRxBuf[i] = 0U;
    }
}