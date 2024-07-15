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

#define TEST_GPMC_FLASH_OFFSET_BASE        (0x200000U)
#define TEST_GPMC_DATA_SIZE                (512U)
#define TEST_GPMC_DATA_REPEAT_COUNT        (8U)
#define TEST_GPMC_BUF_SIZE                 (TEST_GPMC_DATA_SIZE * TEST_GPMC_DATA_REPEAT_COUNT)
#define TEST_CHAR_RANGE                    (256U)


/* FLASH Instance Macros */
#define CONFIG_FLASH0 (0U)
#define CONFIG_FLASH_NUM_INSTANCES (1U)

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

/*
 * FLASH related functions and structures
 */
#include <board/flash.h>

/* FLASH Instance Macros */
#define CONFIG_FLASH0 (0U)
#define CONFIG_FLASH_NUM_INSTANCES (1U)

uint8_t gNandDataScratchMem[4096] __attribute__((aligned(128)));
uint8_t gNandEccScratchMem[256];

/* FLASH Driver handles */
extern Flash_Handle gFlashHandle[CONFIG_FLASH_NUM_INSTANCES];

/* FLASH Object - initialized during Flash_open() */
Flash_NandGpmcObject gFlashObject_MT29F8G08ADAFAH4 =
{
    .gpmcHandle = NULL,
    {
    .eccAlgo = GPMC_NAND_ECC_ALGO_BCH_8BIT,
    },
    .dataMemScratch = gNandDataScratchMem,
    .eccMemScratch = gNandEccScratchMem,
};

/* FLASH Attrs */
Flash_Attrs gFlashAttrs_MT29F8G08ADAFAH4 =
{
    .flashName = "MT29F8G08ADAFAH4",
    .deviceId = 0xD3D0,
    .manufacturerId = 0x2C,
    .flashSize = 1073741824,
    .blockCount = 4096,
    .blockSize = 262144,
    .pageCount = 64,
    .pageSize = 4096,
    .spareAreaSize = 256,
};

/* FLASH DevConfig */
Flash_DevConfig gFlashDevCfg_MT29F8G08ADAFAH4 =
{
    .idCfg = {
        .cmd =0x90,
        .numBytes = 3,
    },
    .eraseCfg = {
        .cmdBlockErase = 0x60,
        .cmdBlockEraseCyc2 = 0xD0,
        .blockSize =  262144,
    },
    .cmdPageLoadCyc1 = 0x00,
    .cmdPageLoadCyc2 = 0x30,
    .cmdRandomReadCyc1 = 0x05,
    .cmdRandomReadCyc2 = 0xE0,
    .cmdRandomInput  = 0x85,
    .cmdPageProgCyc1 = 0x80,
    .cmdPageProgCyc2 = 0x10,
    .pageColAddrCyc = 0x02,
    .pageRowAddrCyc = 0x03,
    .cmdReadStatus = 0x70,
    .cmdReset = 0xFF,
};

/* FLASH Driver handles - opened during Board_flashOpen() */
Flash_Handle gFlashHandle[CONFIG_FLASH_NUM_INSTANCES];

/* FLASH Config */
Flash_Config gFlashConfig[CONFIG_FLASH_NUM_INSTANCES] =
{
    {

        .attrs = &gFlashAttrs_MT29F8G08ADAFAH4,
        .fxns = &gFlashNandGpmcFxns,
        .devConfig = &gFlashDevCfg_MT29F8G08ADAFAH4,
        .object = (void *)&gFlashObject_MT29F8G08ADAFAH4,
    },
};

/* FLASH Open Params - populated from SysConfig options */
Flash_Params gFlashParams[CONFIG_FLASH_NUM_INSTANCES] =
{
    {

        .quirksFxn = NULL,
        .custProtoFxn = NULL,
    },
};

uint32_t gFlashConfigNum = CONFIG_FLASH_NUM_INSTANCES;

void Board_flashClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_FLASH_NUM_INSTANCES; instCnt++)
    {
        if(gFlashHandle[instCnt] != NULL)
        {
            Flash_close(gFlashHandle[instCnt]);
            gFlashHandle[instCnt] = NULL;
        }
    }
    return;
}

int32_t Board_flashOpen()
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_FLASH_NUM_INSTANCES; instCnt++)
    {
        gFlashHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Set the underlying driver instance to the FLASH config */
    gFlashAttrs_MT29F8G08ADAFAH4.driverInstance = CONFIG_GPMC0;

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_FLASH_NUM_INSTANCES; instCnt++)
    {

        gFlashHandle[instCnt] = Flash_open(instCnt, &gFlashParams[instCnt]);
        if(NULL == gFlashHandle[instCnt])
        {
            DebugP_logError("FLASH open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Board_flashClose();   /* Exit gracefully */
    }
    return status;
}

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
    DebugP_log("GPMC read, write and erase test without DMA\r\n");
    Drivers_udmaClose();
    for (i = 0; i < CONFIG_GPMC_NUM_INSTANCES; i++){
        gGpmcParams[i].gpmcDmaChIndex   = -1;
        gGpmcParams[i].dmaEnable        = 0;
    }
    RUN_TEST(test_gpmc_read_write_config, 0, NULL);
    Drivers_gpmcClose();

    /* Test GPMC read, write and erase with dma enabled */
    DebugP_log("GPMC read, write and erase test with DMA\r\n");
    Drivers_gpmcOpen();
    Drivers_udmaOpen();
    for (i = 0; i < CONFIG_GPMC_NUM_INSTANCES; i++){
        gGpmcParams[i].gpmcDmaChIndex   = 0;
        gGpmcParams[i].dmaEnable        = 1;
    }
    RUN_TEST(test_gpmc_read_write_config, 0, NULL);

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
    uint32_t blk, page, i, j;
    uint64_t curTime;
    uint32_t offset = TEST_GPMC_FLASH_OFFSET_BASE;

    /* Open Flash drivers with GPMC instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal  = Board_flashOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Block erase at the test offset */
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    curTime = ClockP_getTimeUsec();
    retVal = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Time taken for Block erase = %" PRId32 " nsec \r\n",
        (uint32_t)(curTime*1000u));

    /* Read back to check for successful block erase */
    for(i = 0; i < TEST_GPMC_DATA_REPEAT_COUNT; i++)
    {
        retVal += Flash_read(gFlashHandle[CONFIG_FLASH0], offset + i*TEST_GPMC_DATA_SIZE, &gGpmcTestRxBuf[(i*TEST_GPMC_DATA_SIZE)], TEST_GPMC_DATA_SIZE);
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    for(i = 0; i < TEST_GPMC_BUF_SIZE; i++)
    {
        if(gGpmcTestRxBuf[i] != 0xFF)
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Initialize GPMC TX buffer */
    for(i = 0; i < TEST_GPMC_BUF_SIZE; i++)
    {
        for(j = 0; j < TEST_GPMC_DATA_SIZE; j++)
        {
            gGpmcTestTxBuf[i] = ((j >= TEST_CHAR_RANGE) ? (TEST_CHAR_RANGE-j) : j);
            i++;
            if(i == TEST_GPMC_BUF_SIZE) 
            {
                break;
            }
        }
    }

    /* GPMC write from TX buffer */
    curTime = ClockP_getTimeUsec();
    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gGpmcTestTxBuf, TEST_GPMC_BUF_SIZE);
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Time taken to program flash for %d bytes of data = %" PRId32 " nsec \r\n",
        TEST_GPMC_BUF_SIZE, (uint32_t)(curTime*1000u));

    /* GPMC read to RX buffer */
    curTime = ClockP_getTimeUsec();
    retVal = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gGpmcTestRxBuf, TEST_GPMC_BUF_SIZE);
    curTime = ClockP_getTimeUsec() - curTime;
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Time taken for flash read = %" PRId32 " nsec for %d bytes of data \r\n",
        (uint32_t)(curTime*1000u),TEST_GPMC_BUF_SIZE);

    /* GPMC compare TX and RX buffers */
    for(i = 0; i < TEST_GPMC_BUF_SIZE; i++)
    {
        if(gGpmcTestRxBuf[i] != gGpmcTestTxBuf[i])
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);


    Board_flashClose();
    Board_driversClose();
}

