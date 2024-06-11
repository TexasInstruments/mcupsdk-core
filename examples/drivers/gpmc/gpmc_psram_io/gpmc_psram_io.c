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

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#define APP_GPMC_PSRAM_OFFSET_BASE  (1024*1024)
#define APP_GPMC_DATA_SIZE          (2*1024*1024)

uint8_t gGpmcTxBuf[APP_GPMC_DATA_SIZE] __attribute__((aligned(64), section("ddr_data"))) ;
/* read buffer MUST be cache line aligned when using DMA, we aligned to 64B though 32B is enough */
uint8_t gGpmcRxBuf[APP_GPMC_DATA_SIZE] __attribute__((aligned(64), section("ddr_data")));

#else
#define APP_GPMC_PSRAM_OFFSET_BASE  (1024*80)
#define APP_GPMC_DATA_SIZE          (1024*40)

uint8_t gGpmcTxBuf[APP_GPMC_DATA_SIZE] ;
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gGpmcRxBuf[APP_GPMC_DATA_SIZE];
#endif

#define NO_OF_ITERATIONS            (2U)

void gpmc_psram_io_fill_buffers();
int32_t gpmc_psram_io_compare_buffers();

void gpmc_psram_io_main(void *args)
{

    int32_t status = SystemP_SUCCESS;
    uint32_t iter = 0;
    uint32_t offset;
    uint64_t startTime, endTime, duration;

    /* Write & read speed in Mbps(Megabits per second)*/
    float writeSpeed = 0;
    float readSpeed = 0;

    /* Open GPMC Driver, among others */
    Drivers_open();
    /* Open Psram drivers with GPMC instance as input */
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    for(iter=0; iter< NO_OF_ITERATIONS; iter++)
    {
        /* Fill buffers with known data,
        * write the data, read back from a specific offset
        * and finally compare the results.
        */
        offset = APP_GPMC_PSRAM_OFFSET_BASE;
        gpmc_psram_io_fill_buffers();

        startTime = ClockP_getTimeUsec();
        Ram_write(gRamHandle[CONFIG_RAM0], offset, gGpmcTxBuf, APP_GPMC_DATA_SIZE);
        endTime = ClockP_getTimeUsec();

        duration = endTime - startTime;
        writeSpeed += ((float)APP_GPMC_DATA_SIZE * 8U)/(duration);

        startTime = ClockP_getTimeUsec();
        Ram_read(gRamHandle[CONFIG_RAM0], offset, gGpmcRxBuf, APP_GPMC_DATA_SIZE);
        endTime = ClockP_getTimeUsec();

        duration = endTime - startTime;
        readSpeed += ((float)APP_GPMC_DATA_SIZE * 8U )/(duration);

        status |= gpmc_psram_io_compare_buffers();

        if(SystemP_SUCCESS != status)
        {
            break;
        }
    }
    if(SystemP_SUCCESS == status)
    {
        writeSpeed = (writeSpeed/NO_OF_ITERATIONS);
        readSpeed = (readSpeed/NO_OF_ITERATIONS);
        DebugP_log("Write Speed: %f Mbps\r\nRead Speed: %f Mbps\r\n",writeSpeed,readSpeed);
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
