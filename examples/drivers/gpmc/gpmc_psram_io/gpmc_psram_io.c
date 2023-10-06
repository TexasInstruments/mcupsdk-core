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

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_GPMC_PSRAM_OFFSET_BASE  0x8000
#define APP_GPMC_DATA_SIZE          (1024*40)

uint8_t gGpmcTxBuf[APP_GPMC_DATA_SIZE] ;
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gGpmcRxBuf[APP_GPMC_DATA_SIZE];

void gpmc_psram_io_fill_buffers();
int32_t gpmc_psram_io_compare_buffers();

void gpmc_psram_io_main(void *args)
{

    int32_t status = SystemP_SUCCESS;
    uint32_t offset;

    /* Open GPMC Driver, among others */
    Drivers_open();
    /* Open Psram drivers with GPMC instance as input */
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    /* Fill buffers with known data,
     * write the data, read back from a specific offset
     * and finally compare the results.
     */
    offset = APP_GPMC_PSRAM_OFFSET_BASE;
    gpmc_psram_io_fill_buffers();

    Psram_write(gPsramHandle[CONFIG_PSRAM0], offset, gGpmcTxBuf, APP_GPMC_DATA_SIZE);
    Psram_read(gPsramHandle[CONFIG_PSRAM0], offset, gGpmcRxBuf, APP_GPMC_DATA_SIZE);
    status |= gpmc_psram_io_compare_buffers();

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

void gpmc_psram_io_fill_buffers()
{
    uint32_t i;

    for(i = 0U; i < APP_GPMC_DATA_SIZE; i++)
    {
        gGpmcTxBuf[i] = i%255;
        gGpmcRxBuf[i] = 0U;
    }
}

int32_t gpmc_psram_io_compare_buffers()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_GPMC_DATA_SIZE; i++)
    {
        if(gGpmcTxBuf[i] != gGpmcRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("GPMC read data mismatch !!!\r\n");
            break;
        }
    }
    return status;
}