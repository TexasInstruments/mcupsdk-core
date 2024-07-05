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
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_OSPI_FLASH_OFFSET_BASE        (0x200000U)
#define TEST_OSPI_FLASH_PHY_TUNING_OFFSET  (0x300000U)
#define TEST_OSPI_DATA_SIZE                (256U) /* has to be 256 B aligned */
#define TEST_OSPI_DATA_REPEAT_COUNT        (8U)
#define TEST_OSPI_RX_BUF_SIZE              (TEST_OSPI_DATA_SIZE * TEST_OSPI_DATA_REPEAT_COUNT)
#if defined(SOC_AM65X)
#define TEST_OSPI_1KB_SIZE                 (256*4U)
#define TEST_OSPI_2KB_SIZE                 (TEST_OSPI_1KB_SIZE*2U)
#define TEST_OSPI_4KB_SIZE                 (TEST_OSPI_1KB_SIZE*4U)
#define TEST_OSPI_1MB_SIZE                 (TEST_OSPI_1KB_SIZE*TEST_OSPI_1KB_SIZE)
#define TEST_OSPI_5MB_SIZE                 (TEST_OSPI_1MB_SIZE*5U)
#define TEST_OSPI_10MB_SIZE                (TEST_OSPI_1MB_SIZE*10U)
#define TEST_OSPI_MAX_TEST_SIZE            (TEST_OSPI_10MB_SIZE)
#define TEST_OSPI_BLOCK_SIZE               (TEST_OSPI_1KB_SIZE*256U)
#define TEST_OSPI_READ_FRCOUNT             (10U)  /* Frequency of reading required for average time of read operation*/
#define TEST_OSPI_PERF_TEST_DATA_COUNT     (3U)   /* Change this value as per testSizes list size */

/* ========================================================================== */
/*                 Structure Declarations                             */
/* ========================================================================== */
/* Structure to store mode settings */
typedef struct Test_FlashModeSettings_t
{
    uint32_t flashType;
    char* flashName;
    uint32_t cfgflashType;
    uint32_t flashProtocol;
    uint32_t phyEnable;
    uint32_t dmaEnable;
    uint32_t dacEnable;
}Test_FlashModeSettings;

typedef struct TestData_SizesAttr_t
{
    uint32_t dataSize; //in MiB
    float writeSpeed;
    float readSpeed;
}TestData_SizesAttr;
#endif

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test cases */
static void test_ospi_read_write_1s1s1s_config(void *args);
static void test_ospi_read_write_max_config(void *args);
static void test_ospi_phy_tuning(void *args);

#if defined(SOC_AM65X)
static void test_ospi_read_perf(void *args);
static float test_ospi_write_in_mb(uint32_t flashOffset, uint32_t writeSize);
static float test_ospi_read_in_mb(uint32_t flashOffset, uint32_t readSize);
static int32_t test_ospi_read_write_test_in_mb(TestData_SizesAttr* testDataCurObj, uint32_t flashOffset, uint32_t dataSize);
static void test_ospi_gdevcfg_set_flash_protocol(uint32_t givenflashProtocol);
static void set_test_flash_type(void);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gOspiTestTxBuf[TEST_OSPI_DATA_SIZE] =
{
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
    0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
    0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,
    0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
    0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,
    0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,
    0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F,
    0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,
    0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,
    0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,
    0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,
    0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,
    0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};

#if defined(SOC_AM65X)
uint8_t gOspiTestTxBulkBuf[TEST_OSPI_MAX_TEST_SIZE]__attribute__ ((section (".globalScratchBuffer"), aligned (128U)));
uint8_t gOspiTestRxBuf[TEST_OSPI_MAX_TEST_SIZE]__attribute__ ((section (".globalScratchBuffer"), aligned (128U)));

static Test_FlashModeSettings modeParams;
#else
uint8_t gOspiTestRxBuf[TEST_OSPI_RX_BUF_SIZE] __attribute__((aligned(128U)));
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
#if defined(SOC_AM65X)
    set_test_flash_type();
    Board_flashClose();
    Drivers_ospiClose();
    Drivers_ospiOpen();
#endif
    UNITY_BEGIN();

    RUN_TEST(test_ospi_read_write_1s1s1s_config, 13386, NULL);
    Drivers_ospiClose();
    Drivers_ospiOpen();
    RUN_TEST(test_ospi_phy_tuning, 13387, NULL);
#if defined(SOC_AM65X)
    Drivers_ospiClose();
    Drivers_ospiOpen();
    RUN_TEST(test_ospi_read_write_max_config, 0, NULL);
    Drivers_ospiClose();
    Drivers_ospiOpen();
    RUN_TEST(test_ospi_read_perf, 0, NULL);
#endif

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
 * Test cases
 */

static void test_ospi_read_write_1s1s1s_config(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t i;
    OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    uint32_t offset = TEST_OSPI_FLASH_OFFSET_BASE;

    /* Initialize the flash device in 1s1s1s mode */
    OSPI_norFlashInit1s1s1s(ospiHandle);

    /* Block erase at the test offset */
    OSPI_norFlashErase(ospiHandle, offset);

    for(i = 0; i < TEST_OSPI_DATA_REPEAT_COUNT; i++)
    {
        OSPI_norFlashWrite(ospiHandle, offset + i*TEST_OSPI_DATA_SIZE, gOspiTestTxBuf, TEST_OSPI_DATA_SIZE);
    }

    OSPI_norFlashRead(ospiHandle, offset, gOspiTestRxBuf, TEST_OSPI_RX_BUF_SIZE);

    for(i = 0; i < TEST_OSPI_RX_BUF_SIZE; i++)
    {
        if(gOspiTestRxBuf[i] != gOspiTestTxBuf[(i%256)])
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}

static void test_ospi_phy_tuning(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t phyTuningData, phyTuningDataSize;
    uint32_t blk, page;
    uint32_t cycles;
    OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Flash the attackVector at a random offset and try the PHY tuning */
    OSPI_phyGetTuningData(&phyTuningData, &phyTuningDataSize);

    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], TEST_OSPI_FLASH_PHY_TUNING_OFFSET, &blk, &page);
    retVal = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], TEST_OSPI_FLASH_PHY_TUNING_OFFSET, (void *)phyTuningData, phyTuningDataSize);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    cycles = CycleCounterP_getCount32();
    retVal = OSPI_phyTuneDDR(ospiHandle, TEST_OSPI_FLASH_PHY_TUNING_OFFSET);
    cycles = CycleCounterP_getCount32() - cycles;

    DebugP_log("TIME FOR PHY TUNING: %d cycles\r \n",cycles);

#if defined(SOC_AM263PX)
    if(cycles > 2000000)
    {
        retVal = SystemP_FAILURE;
    }
#endif

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

#if defined(SOC_AM263PX)
    Flash_reset(gFlashHandle[CONFIG_FLASH0]);
#endif

    Board_driversClose();
}

#if defined(SOC_AM65X)
static void test_ospi_read_write_max_config(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, page, i;
    uint32_t offset = TEST_OSPI_FLASH_OFFSET_BASE;

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Block erase at the test offset */
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
    retVal = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    for(i = 0; i < TEST_OSPI_DATA_REPEAT_COUNT; i++)
    {
        retVal += Flash_write(gFlashHandle[CONFIG_FLASH0], offset + i*TEST_OSPI_DATA_SIZE, gOspiTestTxBuf, TEST_OSPI_DATA_SIZE);
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, gOspiTestRxBuf, TEST_OSPI_RX_BUF_SIZE);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    for(i = 0; i < TEST_OSPI_RX_BUF_SIZE; i++)
    {
        if(gOspiTestRxBuf[i] != gOspiTestTxBuf[(i%256)])
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    Board_driversClose();
}

static void test_ospi_read_perf(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t blk, page;
    uint32_t offset = TEST_OSPI_FLASH_OFFSET_BASE;
    /* Please provide size of atleast 1MiB */
    uint32_t testSizes[TEST_OSPI_PERF_TEST_DATA_COUNT] = {TEST_OSPI_1MB_SIZE, TEST_OSPI_5MB_SIZE, TEST_OSPI_10MB_SIZE};

    TestData_SizesAttr testDataObj[TEST_OSPI_PERF_TEST_DATA_COUNT],TestDataCurrObj;

    const char *flashProtocolList[] = {0,"FLASH_CFG_PROTO_1S_1S_1S","FLASH_CFG_PROTO_1S_1S_2S","FLASH_CFG_PROTO_1S_1S_4S",
                                        "FLASH_CFG_PROTO_1S_1S_8S","FLASH_CFG_PROTO_4S_4S_4S","FLASH_CFG_PROTO_4S_4D_4D",
                                        "FLASH_CFG_PROTO_8S_8S_8S","FLASH_CFG_PROTO_8D_8D_8D","FLASH_CFG_PROTO_CUSTOM"};

    const char *flashTypeList[] = {"SERIAL NOR","SERIAL NAND","PARALLEL NOR","PARALLEL NAND"};

    OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

    modeParams.flashProtocol = gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protocol;
    modeParams.phyEnable = ((OSPI_Config*)ospiHandle)->attrs->phyEnable;
    modeParams.dmaEnable = ((OSPI_Config*)ospiHandle)->attrs->dmaEnable;
    modeParams.dacEnable = ((OSPI_Config*)ospiHandle)->attrs->dacEnable;

    /* Open Flash drivers with OSPI instance as input */
    retVal = Board_driversOpen();

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Block erase at the test offset */
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);

    /* The contents of buffer "gOspiTestTxBuf" are copied at incremental offsets of 'TEST_OSPI_DATA_SIZE'
     * until gOspiTestTxBulkBuf buffer is full
     */
    for(uint32_t txChunkCnt = 0; txChunkCnt < TEST_OSPI_MAX_TEST_SIZE/TEST_OSPI_DATA_SIZE; txChunkCnt++)
    {
        memcpy(gOspiTestTxBulkBuf + txChunkCnt*sizeof(gOspiTestTxBuf) , gOspiTestTxBuf , sizeof(gOspiTestTxBuf));
    }

    for(uint32_t testCount = 0; testCount < sizeof(testSizes)/sizeof(testSizes[0]); testCount++)
    {
        for(uint32_t blkCount = 0; blkCount < testSizes[testCount]/TEST_OSPI_BLOCK_SIZE; blkCount++)
        {
            retVal = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk + blkCount);
            TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        }

        retVal = test_ospi_read_write_test_in_mb(&TestDataCurrObj, offset, testSizes[testCount]);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        memcpy(&testDataObj[testCount],&TestDataCurrObj,sizeof(TestData_SizesAttr));
    }

    /* Print performance numbers. */
    DebugP_log("\n[TEST OSPI] Performance Numbers Print Start\r\n\n");
    DebugP_log("Flash type: %s\r\n",flashTypeList[modeParams.cfgflashType]);
    DebugP_log("Flash protocol: %s\r\n",flashProtocolList[modeParams.flashProtocol]);

    if(modeParams.phyEnable)
        DebugP_log("PHY condition: enabled\r\n");
    else
        DebugP_log("PHY condition: disabled\r\n");

    if(modeParams.dmaEnable)
        DebugP_log("DMA condition: enabled\r\n");
    else
        DebugP_log("DMA condition: disabled\r\n");

    if(modeParams.dacEnable)
        DebugP_log("DAC condition: enabled\r\n");
    else
        DebugP_log("DAC condition: disabled\r\n\n");

    DebugP_log("Data size(MiB) | Write speed(MiBps) | Read speed(MiBps)\r\n");
    DebugP_log("---------------|--------------------|-----------------\r\n");

    for (uint32_t testCount=0; testCount<sizeof(testSizes)/sizeof(testSizes[0]); testCount++) {
        DebugP_log(" %d\t       | %.2f\t\t    | %.2f\r\n", testDataObj[testCount].dataSize, testDataObj[testCount].writeSpeed,
            testDataObj[testCount].readSpeed);
    }

    DebugP_log("\n[TEST OSPI] Performance Numbers Print End\r\n\n");

    Board_driversClose();
}

/*
 * Helper functions
 */

static void set_test_flash_type(void)
{
    modeParams.flashType = gFlashConfig[CONFIG_FLASH0].attrs->flashType;
    modeParams.flashName = gFlashConfig[CONFIG_FLASH0].attrs->flashName;

    /* Entend this for more flashNames when required*/
    if(modeParams.flashType == CONFIG_FLASH_TYPE_SERIAL &&
            ((!strcmp(modeParams.flashName,"S28HS512T")) || (!strcmp(modeParams.flashName,"MT35XU512A"))))
         modeParams.cfgflashType = CONFIG_FLASH_TYPE_SERIAL_NOR;
    else if(modeParams.flashType == CONFIG_FLASH_TYPE_SERIAL && !strcmp(modeParams.flashName,"W35N01JWTBAG"))
         modeParams.cfgflashType = CONFIG_FLASH_TYPE_SERIAL_NAND;
}


static void test_ospi_gdevcfg_set_flash_protocol(uint32_t givenflashProtocol)
{
    if(modeParams.cfgflashType == CONFIG_FLASH_TYPE_SERIAL_NAND)
    {
        switch (givenflashProtocol) {
            case FLASH_CFG_PROTO_1S_1S_1S:
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protocol = FLASH_CFG_PROTO_1S_1S_1S;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.isDtr = FALSE;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.cmdRd = 0x03;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.cmdWr = 0x84;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyClksCmd = 0;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyClksRd = 8;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.isAddrReg = FALSE;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.cmdRegWr = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.cmdRegRd = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.cfgReg = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.shift = 0;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.mask = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.protoCfg.cfgRegBitP = 0;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.isAddrReg = FALSE;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.cmdRegWr = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.cmdRegRd = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.cfgReg = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.shift = 0;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.mask = 0x00;
	            gFlashConfig[CONFIG_FLASH0].devConfig->protocolCfg.dummyCfg.cfgRegBitP = 0;
                break;

            default:
                break;
        }
    }
}

static float test_ospi_write_in_mb(uint32_t flashOffset, uint32_t writeSize)
{
    int32_t retVal = SystemP_SUCCESS;
    uint64_t startTime, endTime;

    startTime = ClockP_getTimeUsec();
    retVal = Flash_write(gFlashHandle[CONFIG_FLASH0], flashOffset, gOspiTestTxBulkBuf, writeSize);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    endTime = ClockP_getTimeUsec();

    return (float)(endTime - startTime);
}

static float test_ospi_read_in_mb(uint32_t flashOffset, uint32_t readSize)
{
    int32_t retVal = SystemP_SUCCESS;
    uint64_t startTime, endTime, totalReadTime=0;
    uint8_t readFcnt = 0;

    while(readFcnt++ < TEST_OSPI_READ_FRCOUNT)
    {
        startTime = ClockP_getTimeUsec();
        retVal = Flash_read(gFlashHandle[CONFIG_FLASH0], flashOffset, gOspiTestRxBuf, readSize);
        endTime = ClockP_getTimeUsec();
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
        totalReadTime += endTime-startTime;
    }
    return (float)totalReadTime/(readFcnt-1);
}

static int32_t test_ospi_read_write_test_in_mb(TestData_SizesAttr* testDataCurObj, uint32_t flashOffset, uint32_t dataSize)
{
    int32_t retVal = SystemP_SUCCESS;
    float readTime, writeTime;

    if(dataSize < TEST_OSPI_1MB_SIZE){
        retVal = SystemP_FAILURE;
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    writeTime = test_ospi_write_in_mb(flashOffset, dataSize);

    readTime = test_ospi_read_in_mb(flashOffset, dataSize);

    testDataCurObj->dataSize = dataSize/TEST_OSPI_1MB_SIZE;
    testDataCurObj->writeSpeed = (float)((float)dataSize/(float)(writeTime));
    testDataCurObj->readSpeed = (float)((float)dataSize/(float)(readTime));

    /* Test if read data matches with written data */
    retVal = memcmp(gOspiTestRxBuf, gOspiTestTxBulkBuf, dataSize);
    return retVal;
}
#endif

