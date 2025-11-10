/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <drivers/qspi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define TEST_QSPI_FLASH_OFFSET      (0x40000U)

#define TEST_QSPI_DATA_SIZE_256_BYTES       (256U)
/* Test data size non-multiple of 256 Bytes. */
#define TEST_QSPI_DATA_SIZE_300_BYTES       (300U)

#define TEST_QSPI_READ_DATA         (512)
#define QPSI_ADDR_LEN_IN_BYTES      (3U)
/* Some common NOR XSPI flash commands */
#define QSPI_NOR_CMD_RDID           (0x9FU)
#define QSPI_NOR_CMD_SINGLE_READ    (0x03U)
#define QSPI_NOR_CMD_DUAL_READ      (0x3BU)
#define QSPI_NOR_CMD_QUAD_READ      (0x6BU)
#define QSPI_NOR_PAGE_PROG          (0x02U)
#define QSPI_NOR_CMD_RSTEN          (0x66U)
#define QSPI_NOR_CMD_RST            (0x99U)
#define QSPI_NOR_CMD_WREN           (0x06U)
#define QSPI_NOR_CMD_WRSR           (0x01U)
#define QSPI_NOR_CMD_RDSR1          (0x05U)
#define QSPI_NOR_CMD_RDSR2          (0x35U)
#define QSPI_NOR_CMD_SECTOR_ERASE   (0x20U)
#define QSPI_NOR_CMD_BLOCK_ERASE    (0xD8U)
#define QSPI_NOR_CMD_RDSFDP         (0x5AU)

#define QSPI_NOR_SR_WIP             (1U << 0U)
#define QSPI_NOR_SR_WEL             (1U << 1U)

#define QSPI_NOR_WRR_WRITE_TIMEOUT  (1200U * 1000U)
#define QSPI_NOR_PAGE_PROG_TIMEOUT  (400U)
#define QSPI_Timeout_10ms           (100000)
#define QSPI_Timeout_20ms           (200000)

#define QSPI_MEM_TYPE_CONFIG        (0U)
#define QSPI_MEM_TYPE_MEM2MEM       (1U)
typedef struct QSPI_TestParams_s {
    /* Test Case Id. */
    uint32_t    testId;
    /* Number of bytes to  be Transmitted or received. */
    uint32_t    transferLength;
    /* Number of rx Lines used for QSPI reading */
    uint32_t    rxLines;
    /* mem type: memory to memory copy / config mode */
    uint32_t    memType;
} QSPI_TestParams;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static int32_t QSPI_norFlashInit(QSPI_Handle handle);
static int32_t QSPI_norFlashCmdRead(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *rxBuf, uint32_t rxLen);
static int32_t QSPI_norFlashCmdWrite(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *txBuf, uint32_t txLen);
static int32_t QSPI_norFlashWriteEnableLatched(QSPI_Handle handle, uint32_t timeOut);
static int32_t QSPI_norFlashWaitReady(QSPI_Handle handle, uint32_t timeOut);
static int32_t QSPI_norFlashWrite(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_norFlashRead(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len, void *args);
static int32_t QSPI_norFlashErase(QSPI_Handle handle, uint32_t address, uint8_t eraseCmd);
static int32_t QSPI_norFlashReadId(QSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId);
static int32_t QSPI_flash_test_compare_buffers(uint32_t len);
static int32_t QSPI_norFlashReadIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_norFlashWriteIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_dataSort(uint8_t *rxBuf, uint8_t bit, uint32_t transferLength);

/* Test cases */
static void test_qspi_config_readWrite(void *args);
static void test_qspi_flashWriteRead(void *args);
static void test_qspi_flashIntrWriteRead(void *args);
static void test_qspi_blockErase(void *args);
static void test_qspi_entireFlashWriteRead(void *args);
static void test_qspi_flashRead16bit(void *args);
static void test_qspi_flashRead32bit(void *args);
static void test_qspi_flashDualRead(void *args);
static void test_qspi_flashPageSize(void *args);

static void qspi_test_fill_buffers(uint32_t len);
static void test_qspi_set_params(QSPI_TestParams *testParams, uint32_t testCaseId);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gQspiTxBuf[TEST_QSPI_DATA_SIZE_256_BYTES];
uint8_t gQspiRxBuf[TEST_QSPI_READ_DATA] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_qspi_main(void *args)
{
    QSPI_TestParams  testParams;
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    test_qspi_set_params(&testParams, 12969); 
    RUN_TEST(test_qspi_config_readWrite, 12969, (void*)&testParams);
    test_qspi_set_params(&testParams, 12969);
    RUN_TEST(test_qspi_blockErase, 12969, (void*)&testParams);
    test_qspi_set_params(&testParams, 12970);
    RUN_TEST(test_qspi_flashWriteRead, 12970, (void*)&testParams);
    test_qspi_set_params(&testParams, 12971);
    RUN_TEST(test_qspi_flashIntrWriteRead, 12971, (void*)&testParams);
    test_qspi_set_params(&testParams, 12974);
    RUN_TEST(test_qspi_flashRead16bit,12974, (void*)&testParams);
    test_qspi_set_params(&testParams, 12994);
    RUN_TEST(test_qspi_flashRead32bit,12994, (void*)&testParams);
    test_qspi_set_params(&testParams, 12993);
    RUN_TEST(test_qspi_flashDualRead, 12993, (void*)&testParams);
    test_qspi_set_params(&testParams, 12988);
    RUN_TEST(test_qspi_flashPageSize, 12988, (void*)&testParams);
    test_qspi_set_params(&testParams, 12972);
    RUN_TEST(test_qspi_entireFlashWriteRead, 12972, (void*)&testParams);
    test_qspi_set_params(&testParams, 14822);
    RUN_TEST(test_qspi_flashIntrWriteRead, 14822, (void*)&testParams);

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

/* Test Description : This API is used to validate the write and read functionality */
static void test_qspi_flashWriteRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;
    
    qspi_test_fill_buffers(tranferLength);

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    (void) QSPI_norFlashInit(qspiHandle);
    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET,QSPI_NOR_CMD_BLOCK_ERASE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* First 256 bytes write in config mode */
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    QSPI_norFlashWaitReady(qspiHandle,400U);

    /* Based on the sysconfig choice , the read will be performed either in memory map or DMA mode*/
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, tranferLength, testParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    QSPI_setMemAddrSpace(qspiHandle,1U);

    status += QSPI_flash_test_compare_buffers(tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

/* Test Description : This API is used to validate the config mode read and write */
static void test_qspi_config_readWrite(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t manfId=0, deviceId=0;
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    (void) QSPI_norFlashInit(qspiHandle);
    /* Read ID */
    status = QSPI_norFlashReadId(qspiHandle, &manfId, &deviceId);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[QSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("[QSPI Flash Diagnostic Test] Flash Device ID       : 0x%X\r\n", deviceId);
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_flashIntrWriteRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength; 
    uint32_t offset = (TEST_QSPI_FLASH_OFFSET + 256);

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    qspi_test_fill_buffers(tranferLength);

    DebugP_log("Executing Flash Erase on first block at offset 0x%x...\r\n", offset);
    QSPI_norFlashErase(qspiHandle, offset, QSPI_NOR_CMD_BLOCK_ERASE);

    QSPI_norFlashWriteIntr(qspiHandle, offset, gQspiTxBuf, tranferLength);

    ClockP_usleep(5000);

    memset(gQspiRxBuf,0,tranferLength);

    QSPI_norFlashReadIntr(qspiHandle, offset, gQspiRxBuf, tranferLength);

    QSPI_setMemAddrSpace(qspiHandle, 1U);

    status += QSPI_flash_test_compare_buffers(tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_blockErase(void *args)
{
    int32_t status = SystemP_SUCCESS;
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET, QSPI_NOR_CMD_BLOCK_ERASE);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_entireFlashWriteRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;
    uint32_t pageSize = 256U;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    qspi_test_fill_buffers(tranferLength);

    /* Flash Size - 8MB in AM273x & AWR294x */
    uint32_t flashSize = 0x7C0000;
    uint32_t itr = 0U;
    uint32_t blocksize = 1U;

    DebugP_log("Executing Flash Erase and writing on entire flash...\r\n");
    DebugP_log("Wait for few minutes...\r\n");
    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET, QSPI_NOR_CMD_BLOCK_ERASE);
    for(itr = TEST_QSPI_FLASH_OFFSET; itr<flashSize; itr = itr + pageSize, blocksize++)
    {
        if(blocksize > pageSize)
        {
            status = QSPI_norFlashErase(qspiHandle, itr,QSPI_NOR_CMD_BLOCK_ERASE);
            blocksize = 1U;
        }
        status = QSPI_norFlashWrite(qspiHandle, itr, gQspiTxBuf, tranferLength);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        status = QSPI_norFlashWaitReady(qspiHandle, QSPI_NOR_WRR_WRITE_TIMEOUT);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }
    DebugP_log("Writing to the flash completed.\r\n");
    QSPI_setMemAddrSpace(qspiHandle, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
    DebugP_log("Reading from the flash, wait for few minutes...\r\n");
    for(itr = TEST_QSPI_FLASH_OFFSET; itr<flashSize; itr = itr + pageSize)
    {
        status = QSPI_norFlashRead(qspiHandle, itr, gQspiRxBuf, tranferLength, testParams);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        status += QSPI_flash_test_compare_buffers(tranferLength);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        memset(gQspiRxBuf,0,tranferLength);
    }
    DebugP_log("Reading from the flash completed.\r\n");
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_flashRead16bit(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readCmd = QSPI_NOR_CMD_SINGLE_READ ; //0x0B;
    QSPILLD_InitHandle      qspilldHandle;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    qspi_test_fill_buffers(tranferLength);
    (void) QSPI_norFlashInit(qspiHandle);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET, QSPI_NOR_CMD_BLOCK_ERASE);
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 16U;
    /* The read is performed in config mode with wordlength as 16 bit*/
    status = QSPI_norFlashCmdRead(qspiHandle, readCmd, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_dataSort(gQspiRxBuf, 16U, tranferLength);
    status += QSPI_flash_test_compare_buffers(tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 8U;
}
static void test_qspi_flashRead32bit(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readCmd = QSPI_NOR_CMD_SINGLE_READ ; //0x0B;
    QSPILLD_InitHandle      qspilldHandle;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    qspi_test_fill_buffers(tranferLength);
    (void) QSPI_norFlashInit(qspiHandle);
 
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 32U;
    /* The read is performed in config mode with wordlength as 32 bit*/
    status = QSPI_norFlashCmdRead(qspiHandle, readCmd, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_dataSort(gQspiRxBuf, 32, tranferLength);
    status += QSPI_flash_test_compare_buffers(tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 8U;
}

static void test_qspi_flashDualRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_InitHandle      qspilldInitHandle;
    QSPILLD_Handle          qspilldHandle;
    uint8_t readCmd = 0U;
    uint32_t dummyBit = 0U;
    uint32_t frameFormat = 0U;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldInitHandle    = &obj->qspilldInitObject;
    qspilldHandle         = &obj->qspilldObject;

    qspi_test_fill_buffers(tranferLength);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET, QSPI_NOR_CMD_BLOCK_ERASE);
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, tranferLength);
 
    /* Configure it in Dual line*/
    qspilldInitHandle->dmaEnable = false;
    qspilldInitHandle->rxLines = QSPI_RX_LINES_DUAL;
    qspilldInitHandle->csPol = QSPI_CS_POL_ACTIVE_LOW;
    readCmd = qspilldHandle->readCmd;
    dummyBit = qspilldHandle->numDummyBits ;
    frameFormat = qspilldInitHandle->frmFmt;
    /* Configure the read command and dummy bits*/
    qspilldHandle->readCmd = 0x3B;
    qspilldHandle->numDummyBits = 8U;
    qspilldInitHandle->frmFmt = QSPI_FF_POL0_PHA0;
    /* Based on the sysconfig choice , the read will be performed either in memory map or DMA mode*/
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, tranferLength, testParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_flash_test_compare_buffers(tranferLength);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);  
    qspilldHandle->readCmd = readCmd;  
    qspilldHandle->numDummyBits  = dummyBit;
    qspilldInitHandle->frmFmt = frameFormat;
    qspilldInitHandle->rxLines = QSPI_RX_LINES_SINGLE;
}

static void test_qspi_flashPageSize(void *args)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_InitHandle      qspilldHandle;
    uint32_t offset;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;
    uint32_t tranferLength = testParams->transferLength;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    qspi_test_fill_buffers(tranferLength);
    (void) QSPI_norFlashInit(qspiHandle);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET, QSPI_NOR_CMD_BLOCK_ERASE);

    /* Write 256 bytes in config mode */
    offset = TEST_QSPI_FLASH_OFFSET;
    status = QSPI_norFlashWrite(qspiHandle, offset, gQspiTxBuf, 250);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Memory Map Read */
    qspilldHandle->dmaEnable = false;
    memset(gQspiRxBuf,0,TEST_QSPI_READ_DATA);
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, 250, testParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_flash_test_compare_buffers(250);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    QSPI_setMemAddrSpace(qspiHandle,1U);
}
/*
*    Internal Function
*/

int32_t QSPI_flash_test_compare_buffers(uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0U;
    uint32_t pageItr = 0U;
    uint32_t bufferLength = len;
    uint32_t txItr = 0U;
    if(len < 256 || len > 256)
    {
        bufferLength = len;
    }
    else 
    {
        bufferLength = 256;
    }

    for (pageItr = 0U; pageItr <= (len/256) ; pageItr = pageItr + 256)
    {
        for(itr = pageItr; itr < bufferLength; itr++,txItr++)
        {
            if(gQspiTxBuf[txItr] != gQspiRxBuf[itr])
            {
                status = SystemP_FAILURE;
                DebugP_logError("Tx %u Rx %u \r\n",gQspiTxBuf[itr],gQspiRxBuf[itr]);
                DebugP_logError("QSPI read data mismatch !!!\r\n");
                break;
            }
        }
    }
    return status;
}

static int32_t QSPI_dataSort(uint8_t *rxBuf, uint8_t bit, uint32_t tranferLength)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0;
    uint8_t dataSort[4];
    if(bit == 16U)
    {
        for(itr = 0U;itr < tranferLength; itr = itr+2)
        {
            *(rxBuf+itr)  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
            *(rxBuf+(itr+1))  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
            *(rxBuf+itr)  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
        }
    }
    else if(bit == 32U)
    {
        for(itr = 0U;itr < tranferLength; itr = itr+4)
        {
            dataSort[0] = (*(rxBuf+(itr+3)));
            dataSort[1] = (*(rxBuf+(itr+2)));
            dataSort[2] = (*(rxBuf+(itr+1)));
            dataSort[3] = (*(rxBuf+(itr)));
            (*(rxBuf+(itr))) = dataSort[0];
            (*(rxBuf+(itr+1))) = dataSort[1];
            (*(rxBuf+(itr+2))) = dataSort[2];
            (*(rxBuf+(itr+3))) = dataSort[3];
        }

    }
    
    return status;
}

int32_t QSPI_norFlashCmdRead(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;

    QSPI_ReadCmdParams rdParams;
    QSPI_readCmdParams_init(&rdParams);
    rdParams.cmd       = cmd;
    rdParams.cmdAddr   = cmdAddr;
    rdParams.rxDataBuf = rxBuf;
    rdParams.rxDataLen = rxLen;

    status += QSPI_readCmd(handle, &rdParams);

    return status;
}

int32_t QSPI_norFlashWriteIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Check offset alignment */
    if(0U != (offset % 256U))
    {
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (handle != NULL) && (buf != NULL) && (len > 0))
    {
        /* As per datasheet max 256 bytes can be programmmed, that is the page size. */
        uint32_t pageSize = 256U;
        uint32_t chunkLen = 0U , actual = 0U;
        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        uint8_t cmrProg = QSPI_NOR_PAGE_PROG;

        qspi_test_fill_buffers(len);

        while ((actual < len) && (status == SystemP_SUCCESS))
        {
            /* Write Enable */
            status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0U);
            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
            }

            if(status == SystemP_SUCCESS)
            {
                /*  Calculate chunk length for this page. To program more than one page:
                 *  You must issue multiple Page Program commands, each for a single page. 
                 */

                chunkLen = ((len - actual) < pageSize) ? (len - actual) : pageSize;

                QSPI_WriteCmdParams wrParams = {0};
                wrParams.cmd = cmrProg;
                wrParams.cmdAddr = offset;
                wrParams.numAddrBytes = QPSI_ADDR_LEN_IN_BYTES;
                wrParams.txDataBuf = (void *)(buf + actual);
                wrParams.txDataLen = chunkLen;

                status = QSPI_writeConfigModeIntr(handle, &wrParams);
            }

            if(status == SystemP_SUCCESS)
            {
                /* Wait for write to complete before next page */
                status = QSPI_norFlashWaitReady(handle, QSPI_NOR_PAGE_PROG_TIMEOUT);
            }

            if(status == SystemP_SUCCESS)
            {
                offset += chunkLen;
                actual += chunkLen;
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

int32_t QSPI_norFlashReadIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_ReadCmdParams rdParams = {0};

    if ((handle != NULL) && (handle->attrs != NULL) && (buf != NULL) && (len > 0U))
    {
        uint32_t rxLines = handle->attrs->rxLines;
        
        /* Send Read Program command */
        if(rxLines == QSPI_RX_LINES_QUAD)
        {
            rdParams.cmd = QSPI_NOR_CMD_QUAD_READ;
        }
        else if(rxLines == QSPI_RX_LINES_DUAL)
        {
            rdParams.cmd = QSPI_NOR_CMD_DUAL_READ;
        }
        else
        {
            rdParams.cmd = QSPI_NOR_CMD_SINGLE_READ;
        }

        rdParams.cmdAddr = offset;
        rdParams.numAddrBytes = QPSI_ADDR_LEN_IN_BYTES;
        rdParams.rxDataBuf = (void *)(buf);
        rdParams.rxDataLen = len;
        status = QSPI_readConfigModeIntr(handle,&rdParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_norFlashCmdWrite(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;

    QSPI_WriteCmdParams wrParams;
    QSPI_writeCmdParams_init(&wrParams);
    wrParams.cmd        = cmd;
    wrParams.cmdAddr    = cmdAddr;
    wrParams.txDataBuf  = txBuf;
    wrParams.txDataLen  = txLen;
    status += QSPI_writeCmd(handle, &wrParams);

    return status;
}


int32_t QSPI_norFlashWriteEnableLatched(QSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    uint8_t cmd;
    uint32_t timeout = timeOut;

    cmd = QSPI_NOR_CMD_RDSR1;

    status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1U);

    while((status == SystemP_SUCCESS) && timeout > 0U)
    {
        status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1U);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WEL) != 0U))
        {
            break;
        }

        timeout--;
    }

    if((readStatus & QSPI_NOR_SR_WEL) != 0)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_norFlashWaitReady(QSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    uint8_t cmd;
    uint32_t timeout = timeOut;

    cmd = QSPI_NOR_CMD_RDSR1;

    status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1U);

    while((status == SystemP_SUCCESS) && timeout > 0U)
    {
        status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1U);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WIP) == 0U))
        {
            break;
        }

        timeout--;
    }

    if((readStatus & QSPI_NOR_SR_WIP) == 0U)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_norFlashInit(QSPI_Handle handle)
{
    uint8_t cmd;

    /* Reset the Flash */
    cmd = QSPI_NOR_CMD_RSTEN;
    (void) QSPI_norFlashCmdWrite(handle, cmd, QSPI_CMD_INVALID_ADDR, NULL, 0U);

    cmd = QSPI_NOR_CMD_RST;
    (void) QSPI_norFlashCmdWrite(handle, cmd, QSPI_CMD_INVALID_ADDR, NULL, 0U);

    (void) QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);

    (void) QSPI_setWriteCmd(handle, QSPI_NOR_PAGE_PROG);

    (void) QSPI_setReadCmd(handle, QSPI_NOR_CMD_SINGLE_READ);

    (void) QSPI_setAddressByteCount(handle, 3);

    (void) QSPI_setDummyBitCount(handle, 0);

    return 0;
}

int32_t QSPI_norFlashWrite(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Check offset alignment */
    if(0U != (offset % 256U))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        uint32_t pageSize, chunkLen, actual;
        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        QSPI_Transaction transaction;

        pageSize = 256;
        chunkLen = pageSize;

        for (actual = 0; actual < len; actual += chunkLen)
        {
            status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0U);

            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
            }
            if(status == SystemP_SUCCESS)
            {
                /* Send Page Program command */
                if((len - actual) < (pageSize))
                {
                    chunkLen = (len - actual);
                }
                else
                {
                    chunkLen = pageSize;
                }

                QSPI_transaction_init(&transaction);
                transaction.addrOffset = offset;
                transaction.buf = (void *)(buf + actual);
                transaction.count = chunkLen;
                status = QSPI_writeConfigMode(handle, &transaction);
            }

            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWaitReady(handle, QSPI_NOR_PAGE_PROG_TIMEOUT);
            }

            if(status == SystemP_SUCCESS)
            {
                offset += chunkLen;
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

int32_t QSPI_norFlashRead(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len, void *args)
{
    int32_t status = SystemP_FAILURE;
    QSPI_TestParams *testParams = (QSPI_TestParams *)args;

    QSPI_Transaction transaction;

    QSPI_transaction_init(&transaction);
    transaction.addrOffset = offset;
    transaction.buf = (void *)buf;
    transaction.count = len;
    transaction.transferTimeout = QSPI_Timeout_10ms;
    if(testParams->memType == QSPI_MEM_TYPE_MEM2MEM)
    {
        status = QSPI_readMemMapMode(handle, &transaction);
    }
    else
    {
        status = QSPI_readConfigMode(handle, &transaction);
    }

    return status;
}

int32_t QSPI_norFlashErase(QSPI_Handle handle, uint32_t address, uint8_t eraseCmd)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmdWren = QSPI_NOR_CMD_WREN;
    uint8_t cmd;

    cmd    = eraseCmd;

    status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0U);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashCmdWrite(handle, cmd, address, NULL, 0U);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }

    return status;
}

int32_t QSPI_norFlashReadId(QSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmd = QSPI_NOR_CMD_RDID;
    uint8_t idCode[3] = { 0 };

    status += QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, idCode, 3U);

    if(status == SystemP_SUCCESS)
    {
        *manufacturerId = (uint32_t)idCode[0];
        *deviceId = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
    }

    return status;
}

static void qspi_test_fill_buffers(uint32_t len)
{
    uint32_t i = 0U;
    for (i = 0U; i < len; i++)
    {
        gQspiTxBuf[i] = (uint8_t)i;
        gQspiRxBuf[i] = 0U;
    }
}

static void test_qspi_set_params(QSPI_TestParams *testParams, uint32_t tcId)
{
    testParams->testId = tcId;
    /* default parameters. */
    testParams->memType = QSPI_MEM_TYPE_MEM2MEM;
    testParams->rxLines = QSPI_RX_LINES_SINGLE;
    testParams->transferLength = TEST_QSPI_DATA_SIZE_256_BYTES;
    switch (tcId)
    {
        case 14822:
            testParams->rxLines = QSPI_RX_LINES_QUAD;
            testParams->transferLength = TEST_QSPI_DATA_SIZE_300_BYTES;
            break;    
        case 12969:
            testParams->transferLength = TEST_QSPI_DATA_SIZE_300_BYTES;
            break;
        case 12970:
            testParams->transferLength = TEST_QSPI_DATA_SIZE_256_BYTES;
            break;
        case 12971:
            break;
        case 12974:
            testParams->transferLength = TEST_QSPI_DATA_SIZE_256_BYTES;
            break;
        case 12994:
            testParams->transferLength = TEST_QSPI_DATA_SIZE_256_BYTES;
            break;
        case 12993:
            testParams->rxLines = QSPI_RX_LINES_DUAL;
            break;
        case 12988:
            break;
        case 12972:
            break;
        default:
            break;
    }
}