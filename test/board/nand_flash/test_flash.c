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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/pmu.h>
#include "drivers/pmu/pmu.c"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define TEST_FLASH_OFFSET_BASE      (0x10000U)
#else
#define TEST_FLASH_OFFSET_BASE      (0x200000U)
#endif
#define TEST_FLASH_DATA_SIZE        (1024U)
#define TEST_FLASH_TX_BUF_SIZE      (2048U)
#define TEST_FLASH_RX_BUF_SIZE      (2048U)
#define TEST_FLASH_BUF_SIZE_BLOCK   (128*1024U)
#define TEST_FLASH_BUF_LEN_ODD      (15U)
#define TEST_FLASH_BYTE_OFFSET_ODD  (7U)
#define TEST_FLASH_TEMP_BUF_SIZE    (32U)
#define TEST_FLASH_BUF_LEN_ODD_DMA  (2021U)
#define NUM_OF_ITERATIONS           (1U)
#define CHIP_SEL_PIN (37)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

uint8_t gFlashTestTxBuf[TEST_FLASH_TX_BUF_SIZE];

uint8_t gFlashTestRxBuf[TEST_FLASH_RX_BUF_SIZE] __attribute__((aligned(128U)));

uint8_t gFlashTestBuf[TEST_FLASH_BUF_SIZE_BLOCK] __attribute__((aligned(128U)));

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */


/**
 * @brief wirte one complete page of 2KB in block 0 and then read it back and then compare
 *
 * @param args
 */
static void test_flash_readwrite_fullpage(void *args);
/**
 * @brief wirte 2Kb of data across 2 pages without any discontinuity of block 16
 *
 * @param args
 */
static void test_flash_readwrite_halfpage(void *args);

static void test_flash_readwrite_block(void *args);

static void test_flash_readwrite_1MB(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

PMU_ProfileObject gProfileObject;

PMU_EventCfg gPmuEventCfg[3] =
{
    {
        .name = "ICache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS,
    },
    {
        .name = "DCache Access",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_ACCESS,
    },
    {
        .name = "DCache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_MISS,
    },
};

PMU_Config gPmuConfig =
{
    .bCycleCounter = TRUE,
    .numEventCounters = 3U,
    .eventCounters = gPmuEventCfg,
};


void nand_flash_chip_select(uint32_t gpioAddress,uint32_t pinNum)
{
    GPIO_setDirMode(gpioAddress, pinNum, 0);

    GPIO_pinWriteLow(gpioAddress, pinNum);
}

void getSpeed(uint32_t index,uint32_t dataSize)
{
    uint64_t clk_frequency = SOC_getSelfCpuClk();
    PMU_ProfilePoint *p = &gProfileObject.point[index];
    uint32_t cycleCount = p->cycleCount.value;
    uint64_t speed = ((float)clk_frequency/cycleCount)*dataSize;

    DebugP_log("Speed in Bps %lld \r \n",speed);
}

void test_main(void *args)
{
    UNITY_BEGIN();

    uint32_t    gpioBaseAddr, pinNum;
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_GPIO0_U_BASE);
    pinNum       = CHIP_SEL_PIN;

    /* Open OSPI and other drivers */

    //test1
    Drivers_open();
    PMU_init(&gPmuConfig);

    nand_flash_chip_select(gpioBaseAddr,pinNum);

    RUN_TEST(test_flash_readwrite_fullpage, 12256, NULL);

    /* Close OSPI and other drivers */
    Drivers_close();

    //test2
    Drivers_open();
    PMU_init(&gPmuConfig);

    nand_flash_chip_select(gpioBaseAddr,pinNum);

    RUN_TEST(test_flash_readwrite_halfpage, 12257, NULL);

    /* Close OSPI and other drivers */
    Drivers_close();

    //test3
    Drivers_open();
    PMU_init(&gPmuConfig);

    nand_flash_chip_select(gpioBaseAddr,pinNum);

    RUN_TEST(test_flash_readwrite_block, 12258, NULL);

    /* Close OSPI and other drivers */
    Drivers_close();

    //test4
    Drivers_open();
    PMU_init(&gPmuConfig);

    nand_flash_chip_select(gpioBaseAddr,pinNum);

    RUN_TEST(test_flash_readwrite_1MB, 12259, NULL);

    /* Close OSPI and other drivers */
    Drivers_close();

    UNITY_END();
    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Testcases*/
static void test_flash_readwrite_fullpage(void *args)
{
    // flash open
    // flash close
    // flash erase
    // flash reset
    // flash read
    // flash write

    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, offset, page;

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // flash reset
    retVal += Flash_reset(gFlashHandle[CONFIG_FLASH0]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    offset = TEST_FLASH_OFFSET_BASE;

    retVal += Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // block erase
    retVal += Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // buffer init

    for(uint32_t i=0;i<TEST_FLASH_TX_BUF_SIZE;i++)
    {
       gFlashTestTxBuf[i] = (i%256);
       gFlashTestRxBuf[i] = 0;
    }

    //flash write

    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestTxBuf, TEST_FLASH_TX_BUF_SIZE);

    if(SystemP_SUCCESS != retVal)
    {
        DebugP_log("[Flash Test] Flash_write failed for offset 0x%X !!!\r\n", offset);
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // flash read back
    retVal += Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestRxBuf, TEST_FLASH_RX_BUF_SIZE);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // compare
    for(uint32_t i=0;i<TEST_FLASH_RX_BUF_SIZE;i++)
    {
        if(gFlashTestRxBuf[i] != gFlashTestTxBuf[i])
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    Board_driversClose();
}

static void test_flash_readwrite_halfpage(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, offset1, offset2, page;

    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);


    // flash reset
    retVal += Flash_reset(gFlashHandle[CONFIG_FLASH0]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    blk=16;
    page=1;

    retVal += Flash_blkPageToOffset(gFlashHandle[CONFIG_FLASH0],&offset1,blk,page);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    page=2;

    retVal += Flash_blkPageToOffset(gFlashHandle[CONFIG_FLASH0],&offset2,blk,page);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    offset2 = offset2 + TEST_FLASH_TX_BUF_SIZE;

     // block erase
    retVal += Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    uint32_t i;

    for(i=0;i<TEST_FLASH_TX_BUF_SIZE;i++)
    {
        gFlashTestTxBuf[i] = (i%256);
        gFlashTestRxBuf[i] = 0;
    }

    //flash write

    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], offset1, gFlashTestTxBuf, TEST_FLASH_DATA_SIZE);

    if(SystemP_SUCCESS != retVal)
    {
        DebugP_log("[Flash Test] Flash_write failed for offset 0x%X !!!\r\n", offset1);
    }

    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], offset2, (uint8_t *)( gFlashTestTxBuf + TEST_FLASH_DATA_SIZE ), TEST_FLASH_DATA_SIZE);

    if(SystemP_SUCCESS != retVal)
    {
        DebugP_log("[Flash Test] Flash_write failed for offset 0x%X !!!\r\n", offset2);
    }

    // read back

    retVal = Flash_read(gFlashHandle[CONFIG_FLASH0], offset1, gFlashTestRxBuf, TEST_FLASH_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = Flash_read(gFlashHandle[CONFIG_FLASH0], offset2,  (uint8_t *)( gFlashTestRxBuf + TEST_FLASH_DATA_SIZE ), TEST_FLASH_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);


    // compare
    for(i=0;i<TEST_FLASH_RX_BUF_SIZE;i++)
    {
        if(gFlashTestRxBuf[i]!=gFlashTestTxBuf[i])
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    Board_driversClose();
}

static void test_flash_readwrite_block(void *args){

    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, offset, page;

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // flash reset
    retVal += Flash_reset(gFlashHandle[CONFIG_FLASH0]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    offset = 0;

    retVal += Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // block erase
    retVal += Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
    {
       gFlashTestBuf[i] = (i%256);
    }

    //flash write
    retVal += Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestBuf, TEST_FLASH_BUF_SIZE_BLOCK);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
    {
       gFlashTestBuf[i] = 0;
    }

    //flash read back
    retVal += Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestBuf, TEST_FLASH_BUF_SIZE_BLOCK);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // compare
    for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
    {
        if(gFlashTestBuf[i]!=(i%256))
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    Board_driversClose();
}

static void test_flash_readwrite_1MB(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, offset, page;

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    // flash reset
    retVal += Flash_reset(gFlashHandle[CONFIG_FLASH0]);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    offset = 0;

    for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
    {
       gFlashTestBuf[i] = (i%256);
    }

    for(uint32_t i=0;i<8;i++)
    {
        offset += i*TEST_FLASH_BUF_SIZE_BLOCK;
        retVal += Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

        // block erase
        retVal += Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

        //flash write
        retVal += Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestBuf, TEST_FLASH_BUF_SIZE_BLOCK);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    offset = 0;

    for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
    {
       gFlashTestBuf[i] = 0;
    }

    for(uint32_t i=0;i<8;i++)
    {
        offset += i*TEST_FLASH_BUF_SIZE_BLOCK;

        //flash read back
        retVal += Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gFlashTestBuf, TEST_FLASH_BUF_SIZE_BLOCK);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

        // compare
        for(uint32_t i=0;i<TEST_FLASH_BUF_SIZE_BLOCK;i++)
        {
            if(gFlashTestBuf[i]!=(i%256))
            {
                retVal = SystemP_FAILURE;
                break;
            }
            gFlashTestBuf[i]=0;
        }

        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    Board_driversClose();
}