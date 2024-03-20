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

#define TEST_QSPI_DATA_SIZE         (256)

#define TEST_QSPI_READ_DATA         (512)

/* Some common NOR XSPI flash commands */
#define QSPI_NOR_CMD_RDID           (0x9FU)
#define QSPI_NOR_CMD_SINGLE_READ    (0x03U)
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

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static int32_t QSPI_norFlashInit(QSPI_Handle handle);
static int32_t QSPI_norFlashCmdRead(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *rxBuf, uint32_t rxLen);
static int32_t QSPI_norFlashCmdWrite(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *txBuf, uint32_t txLen);
static int32_t QSPI_norFlashWriteEnableLatched(QSPI_Handle handle, uint32_t timeOut);
static int32_t QSPI_norFlashWaitReady(QSPI_Handle handle, uint32_t timeOut);
static int32_t QSPI_norFlashWrite(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_norFlashRead(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_norFlashErase(QSPI_Handle handle, uint32_t address, uint8_t eraseCmd);
static int32_t QSPI_norFlashReadId(QSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId);
static int32_t QSPI_flash_test_compare_buffers(uint32_t len);
static int32_t QSPI_norFlashReadIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t QSPI_norFlashWriteIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
int32_t QSPI_dataSort(uint8_t *rxBuf, uint8_t bit);

/* Test cases */

static void test_qspi_config_readWrite(void *args);
static void test_qspi_flashWriteRead(void *args);
static void test_qspi_flashInterruptWriteRead(void *args);
static void test_qspi_blockErase(void *args);
static void test_qspi_entireFlashWriteRead(void *args);
static void test_qspi_flashRead16bit(void *args);
static void test_qspi_flashRead32bit(void *args);
static void test_qspi_flashDualRead(void *args);
static void test_qspi_flashPageSize(void *args);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gQspiTxBuf[TEST_QSPI_DATA_SIZE] =
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

uint8_t gQspiRxBuf[TEST_QSPI_READ_DATA] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_qspi_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_qspi_config_readWrite, 12969, NULL);

    RUN_TEST(test_qspi_blockErase, 12969, NULL);

    RUN_TEST(test_qspi_flashWriteRead, 12970, NULL);

    RUN_TEST(test_qspi_flashInterruptWriteRead, 12971, NULL);

    RUN_TEST(test_qspi_flashRead16bit,12974,NULL);

    RUN_TEST(test_qspi_flashRead32bit,12994,NULL);

    RUN_TEST(test_qspi_flashDualRead,12993,NULL);

    RUN_TEST(test_qspi_flashPageSize,12988,NULL);

    RUN_TEST(test_qspi_entireFlashWriteRead,12972,NULL);

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
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    (void) QSPI_norFlashInit(qspiHandle);
    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET,QSPI_NOR_CMD_BLOCK_ERASE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* First 256 bytes write in config mode */
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    QSPI_norFlashWaitReady(qspiHandle,400U);

    /* Based on the sysconfig choice , the read will be performed either in memory map or DMA mode*/
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    QSPI_setMemAddrSpace(qspiHandle,1U);

    status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
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


static void test_qspi_flashInterruptWriteRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    QSPI_norFlashWriteIntr(qspiHandle, (TEST_QSPI_FLASH_OFFSET + 256), gQspiTxBuf, TEST_QSPI_DATA_SIZE);

    ClockP_usleep(5000);

    memset(gQspiRxBuf,0,TEST_QSPI_DATA_SIZE);

    QSPI_norFlashReadIntr(qspiHandle, (TEST_QSPI_FLASH_OFFSET + 256), gQspiRxBuf, TEST_QSPI_DATA_SIZE);

    QSPI_setMemAddrSpace(qspiHandle,1U);

    status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_blockErase(void *args)
{
    int32_t status = SystemP_SUCCESS;
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET,QSPI_NOR_CMD_BLOCK_ERASE);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_entireFlashWriteRead(void *args)
{
    int32_t status = SystemP_SUCCESS;
    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    /* Flash Size - 8MB in AM273x & AWR294x */
    uint32_t flashSize = 0x7C0000;
    uint32_t itr = 0U;
    uint32_t blocksize = 1U;

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET,QSPI_NOR_CMD_BLOCK_ERASE);
    for(itr = TEST_QSPI_FLASH_OFFSET; itr<flashSize; itr = itr + 256,blocksize++)
    {
        if(blocksize > 256)
        {
            status = QSPI_norFlashErase(qspiHandle, itr,QSPI_NOR_CMD_BLOCK_ERASE);
            blocksize = 1U;
        }
        status = QSPI_norFlashWrite(qspiHandle, itr, gQspiTxBuf, TEST_QSPI_DATA_SIZE);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        status = QSPI_norFlashWaitReady(qspiHandle, QSPI_NOR_WRR_WRITE_TIMEOUT);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    }
    QSPI_setMemAddrSpace(qspiHandle,1U);
    for(itr = TEST_QSPI_FLASH_OFFSET; itr<flashSize; itr = itr + 256)
    {
        status = QSPI_norFlashRead(qspiHandle, itr, gQspiRxBuf, TEST_QSPI_DATA_SIZE);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        memset(gQspiRxBuf,0,TEST_QSPI_DATA_SIZE);
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_qspi_flashRead16bit(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readCmd = QSPI_NOR_CMD_SINGLE_READ ; //0x0B;
    QSPILLD_InitHandle      qspilldHandle;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    memset(gQspiRxBuf,0,TEST_QSPI_DATA_SIZE);
 
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 16U;
    /* The read is performed in config mode with wordlength as 16 bit*/
    status = QSPI_norFlashCmdRead(qspiHandle, readCmd, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_dataSort(gQspiRxBuf,16U);
    status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 8U;
}
static void test_qspi_flashRead32bit(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readCmd = QSPI_NOR_CMD_SINGLE_READ ; //0x0B;
    QSPILLD_InitHandle      qspilldHandle;

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    memset(gQspiRxBuf,0,TEST_QSPI_DATA_SIZE);
 
    status = QSPI_norFlashWrite(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiTxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    qspilldHandle->wrdLen = 32U;
    /* The read is performed in config mode with wordlength as 32 bit*/
    status = QSPI_norFlashCmdRead(qspiHandle, readCmd, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_dataSort(gQspiRxBuf,32);
    status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
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

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldInitHandle    = &obj->qspilldInitObject;
    qspilldHandle         = &obj->qspilldObject;

    memset(gQspiRxBuf,0,TEST_QSPI_DATA_SIZE);
 
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
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, TEST_QSPI_DATA_SIZE);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status += QSPI_flash_test_compare_buffers(TEST_QSPI_DATA_SIZE);
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

    /* Get QSPI Handle */
    QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
    QSPI_Object *obj = ((QSPI_Config *)qspiHandle)->object;
    qspilldHandle    = &obj->qspilldInitObject;

    (void) QSPI_norFlashInit(qspiHandle);

    status = QSPI_norFlashErase(qspiHandle, TEST_QSPI_FLASH_OFFSET,QSPI_NOR_CMD_BLOCK_ERASE);

    /* Write 256 bytes in config mode */
    offset = TEST_QSPI_FLASH_OFFSET;
    status = QSPI_norFlashWrite(qspiHandle, offset, gQspiTxBuf, 250);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Memory Map Read */
    qspilldHandle->dmaEnable = false;
    memset(gQspiRxBuf,0,TEST_QSPI_READ_DATA);
    status = QSPI_norFlashRead(qspiHandle, TEST_QSPI_FLASH_OFFSET, gQspiRxBuf, 250);
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

int32_t QSPI_dataSort(uint8_t *rxBuf, uint8_t bit)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0;
    uint8_t dataSort[4];
    if(bit == 16U)
    {
        for(itr = 0U;itr < TEST_QSPI_DATA_SIZE; itr = itr+2)
        {
            *(rxBuf+itr)  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
            *(rxBuf+(itr+1))  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
            *(rxBuf+itr)  = *(rxBuf+itr) ^ *(rxBuf+(itr+1));
        }
    }
    else if(bit == 32U)
    {
        for(itr = 0U;itr < TEST_QSPI_DATA_SIZE; itr = itr+4)
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
    if(status == SystemP_SUCCESS)
    {

        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        uint8_t cmrProg = QSPI_NOR_PAGE_PROG;
        QSPI_WriteCmdParams wrParams = {0};

        status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0U);

        if(status == SystemP_SUCCESS)
        {
            status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
        }
        if(status == SystemP_SUCCESS)
        {
                /* Send Page Program command */
                wrParams.cmd = cmrProg;
                wrParams.cmdAddr = offset;
                wrParams.numAddrBytes = 3U;
                wrParams.txDataBuf = (void *)(buf);
                wrParams.txDataLen = len;
                status = QSPI_writeConfigModeIntr(handle, &wrParams);
        }
    }

    return status;
}

int32_t QSPI_norFlashReadIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_ReadCmdParams rdParams = {0};

    /* Send Read Program command */
    rdParams.cmd = QSPI_NOR_CMD_SINGLE_READ;
    rdParams.cmdAddr = offset;
    rdParams.numAddrBytes = 3U;
    rdParams.rxDataBuf = (void *)(buf);
    rdParams.rxDataLen = len;
    status = QSPI_readConfigModeIntr(handle,&rdParams);
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

int32_t QSPI_norFlashRead(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_FAILURE;

    QSPI_Transaction transaction;

    QSPI_transaction_init(&transaction);
    transaction.addrOffset = offset;
    transaction.buf = (void *)buf;
    transaction.count = len;
    transaction.transferTimeout = QSPI_Timeout_10ms;
    status = QSPI_readMemMapMode(handle, &transaction);

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
