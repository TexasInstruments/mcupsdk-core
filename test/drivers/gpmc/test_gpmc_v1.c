/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_GPMC_PSRAM_OFFSET_BASE        (0x000000U)
#define TEST_GPMC_DATA_SIZE                (256U)
#define TEST_GPMC_DATA_REPEAT_COUNT        (8U)
#define TEST_GPMC_BUF_SIZE                 (TEST_GPMC_DATA_SIZE * TEST_GPMC_DATA_REPEAT_COUNT)
#define TEST_CHAR_RANGE                    (256U)
#define TEST_ADD_INCREMENT                 (0x100000)
#define TEST_ITERATIONS                    (8)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test case */
static void test_gpmc_read_write_config(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gGpmcTestTxBuf[TEST_GPMC_BUF_SIZE];
uint8_t gGpmcTestRxBuf[TEST_GPMC_BUF_SIZE] __attribute__((aligned(128U)));

uint32_t  gGpmcTestAddIncrement[TEST_ITERATIONS] = {0x00000000, 0x00100000, 0x00200000, 0x00300000, 0x00400000, 0x00500000, 0x00600000, 0x00700000};

/* RAM Driver handles */
extern Ram_Handle gRamHandle[CONFIG_RAM_NUM_INSTANCES];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
extern GPMC_Params gGpmcParams[CONFIG_GPMC_NUM_INSTANCES];
void test_main(void *args)
{
    uint32_t i = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    /* Test GPMC read, write and erase without dma */
    DebugP_log("GPMC read and write test without DMA\r\n");
    for (i = 0; i < CONFIG_GPMC_NUM_INSTANCES; i++){
        gGpmcParams[i].gpmcDmaChIndex   = -1;
        gGpmcParams[i].dmaEnable        = 0;
    }
    RUN_TEST(test_gpmc_read_write_config, 0, NULL);
    Drivers_gpmcClose();

    UNITY_END();
    Drivers_close();

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Test case
 */

static void test_gpmc_read_write_config(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t i, j;
    uint64_t curTime;
    uint32_t offset = TEST_GPMC_PSRAM_OFFSET_BASE;

    /* Open Psram drivers with GPMC instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Initialize GPMC TX buffer */
    for(i = 0; i < TEST_GPMC_BUF_SIZE; i++)
    {
        for(j = 0; j < TEST_GPMC_DATA_SIZE; j++)
        {
            gGpmcTestTxBuf[i] = ((j >= TEST_CHAR_RANGE) ? (TEST_CHAR_RANGE-j) : j);
            i++;
        }
    }

    for(i = 0; i < TEST_ITERATIONS; i++)
    {
        /* GPMC write from TX buffer */
        curTime = ClockP_getTimeUsec();
        retVal = Ram_write(gRamHandle[CONFIG_RAM0], offset + gGpmcTestAddIncrement[i], gGpmcTestTxBuf, TEST_GPMC_BUF_SIZE);
        curTime = ClockP_getTimeUsec() - curTime;
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

        /* GPMC read to RX buffer */
        curTime = ClockP_getTimeUsec();
        retVal = Ram_read(gRamHandle[CONFIG_RAM0], offset + gGpmcTestAddIncrement[i], gGpmcTestRxBuf, TEST_GPMC_BUF_SIZE);
        curTime = ClockP_getTimeUsec() - curTime;
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

        /* GPMC compare TX and RX buffers */
        for(j = 0; j < TEST_GPMC_BUF_SIZE; j++)
        {
            if(gGpmcTestRxBuf[j] != gGpmcTestTxBuf[j])
            {
                retVal = SystemP_FAILURE;
                break;
            }
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    Board_driversClose();
    Drivers_close();
}

