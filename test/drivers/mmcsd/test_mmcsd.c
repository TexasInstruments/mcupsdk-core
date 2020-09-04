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

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_MMCSD_EMMC_START_BLK        (0x300000U) /* 1.5GB */
#define TEST_MMCSD_SD_START_BLK          (0x300000U) /* 1.5GB */
#define TEST_MMCSD_DATA_SIZE             (1024U) /* has to be 256 B aligned */
#define TEST_MMCSD_FILE_LINE_CNT         (100U)
#define TEST_MMCSD_FAT_PARTITION_SIZE    (128U * 1024U * 1024U) /* 128 MB */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void test_mmcsd_fill_buffers(void);
static void test_mmcsd_emmc_raw_io(void *args);
static void test_mmcsd_sd_raw_io(void *args);
static void test_mmcsd_emmc_file_io(void *args);
static void test_mmcsd_sd_file_io(void *args);

static int32_t test_mmcsd_raw_io(MMCSD_Handle handle);
static int32_t test_mmcsd_file_io(char *fileName, char* fileData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gMmcsdTestTxBuf[TEST_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));
uint8_t gMmcsdTestRxBuf[TEST_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_mmcsd_emmc_raw_io, 2069, NULL);
    RUN_TEST(test_mmcsd_sd_raw_io, 2072, NULL);
    RUN_TEST(test_mmcsd_emmc_file_io, 2067, NULL);
    RUN_TEST(test_mmcsd_sd_file_io, 2066, NULL);

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
static void test_mmcsd_emmc_raw_io(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD_EMMC];

    retVal = test_mmcsd_raw_io(handle);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
};

static void test_mmcsd_sd_raw_io(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    MMCSD_Handle handle = gMmcsdHandle[CONFIG_MMCSD_SD];

    retVal = test_mmcsd_raw_io(handle);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
};

static void test_mmcsd_emmc_file_io(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    /* Create partition if not present */
    FF_Disk_t *pDisk = &gFFDisks[FF_PARTITION_EMMC0];

    FF_MMCSD_PartitionDetails partitionDetails;

    FF_MMCSDGetPartitionDetails(pDisk, &partitionDetails);

    if(partitionDetails.sectorCount == 0U)
    {
        /* No partition found, create a `TEST_MMCSD_FAT_PARTITION_SIZE` partition */
        uint32_t blockSize = MMCSD_getBlockSize(gMmcsdHandle[CONFIG_MMCSD_EMMC]);
        uint32_t partSectorCount = TEST_MMCSD_FAT_PARTITION_SIZE / blockSize;

        FF_MMCSDCreateAndFormatPartition(pDisk, partSectorCount);

        /* Now mount the partition */
        FF_MMCSDMountPartition(pDisk, "/emmc0");

        /* Finally check the partition again */
        FF_MMCSDGetPartitionDetails(pDisk, &partitionDetails);

        if(partitionDetails.sectorCount > 0U)
        {
            /* Partition successfully mounted, continue */
        }
        else
        {
            retVal = SystemP_FAILURE;
        }
    }
    else
    {
        /* FAT partition found, all good. Proceed with file I/O testing */
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    char *fileName = "/emmc0/test.dat";
    char *fileData = "THIS IS A TEST FILE TO TEST SD CARD FILE IO\n";

    test_mmcsd_file_io(fileName, fileData);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
};

static void test_mmcsd_sd_file_io(void *args)
{
	int32_t retVal = SystemP_SUCCESS;

    /* Create partition if not present */
    FF_Disk_t *pDisk = &gFFDisks[FF_PARTITION_SD0];

    FF_MMCSD_PartitionDetails partitionDetails;

    FF_MMCSDGetPartitionDetails(pDisk, &partitionDetails);

    if(partitionDetails.sectorCount == 0U)
    {
        /* No partition found, create a `TEST_MMCSD_FAT_PARTITION_SIZE` partition */
        uint32_t blockSize = MMCSD_getBlockSize(gMmcsdHandle[CONFIG_MMCSD_SD]);
        uint32_t partSectorCount = TEST_MMCSD_FAT_PARTITION_SIZE / blockSize;

        FF_MMCSDCreateAndFormatPartition(pDisk, partSectorCount);

        /* Now mount the partition */
        FF_MMCSDMountPartition(pDisk, "/sd0");

        /* Finally check the partition again */
        FF_MMCSDGetPartitionDetails(pDisk, &partitionDetails);

        if(partitionDetails.sectorCount > 0U)
        {
            /* Partition successfully mounted, continue */
        }
        else
        {
            retVal = SystemP_FAILURE;
        }
    }
    else
    {
        /* FAT partition found, all good. Proceed with file I/O testing */
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    char *fileName = "/sd0/test.dat";
    char *fileData = "THIS IS A TEST FILE TO TEST SD CARD FILE IO\n";

    test_mmcsd_file_io(fileName, fileData);

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
};

void test_mmcsd_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < TEST_MMCSD_DATA_SIZE; i++)
    {
        gMmcsdTestTxBuf[i] = i % 256;
        gMmcsdTestRxBuf[i] = 0U;
    }
}

static int32_t test_mmcsd_raw_io(MMCSD_Handle handle)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t i;

    uint32_t blockSize = MMCSD_getBlockSize(handle);
    uint32_t numBlocksPerIter = TEST_MMCSD_DATA_SIZE / blockSize;

    /* write about 16 MB of data and read it back */
    test_mmcsd_fill_buffers();

    uint32_t writeReadLen = 64U*1024U*1024U;
    uint32_t totalNumBlocks = (writeReadLen / blockSize);
    uint32_t iterCount = (totalNumBlocks / numBlocksPerIter);

    for(i = 0; i < iterCount; i++)
    {
        retVal = MMCSD_write(handle, gMmcsdTestTxBuf, TEST_MMCSD_EMMC_START_BLK, numBlocksPerIter);
        if(retVal != SystemP_SUCCESS)
        {
            break;
        }
    }

    if(SystemP_SUCCESS == retVal)
    {
        uint32_t breakLoop = FALSE;
        /* Now read back and verify in chunks */
        for(i = 0; i < iterCount; i++)
        {
            retVal = MMCSD_read(handle, gMmcsdTestRxBuf, TEST_MMCSD_EMMC_START_BLK, numBlocksPerIter);
            
            if(SystemP_SUCCESS == retVal)
            {
                retVal = memcmp(gMmcsdTestRxBuf, gMmcsdTestTxBuf, TEST_MMCSD_DATA_SIZE);

                if(SystemP_SUCCESS != retVal)
                {
                    breakLoop = TRUE;
                }
            }
            else
            {
                breakLoop = TRUE;
            }

            if(breakLoop == TRUE)
            {
                break;
            }
        }
    }

    return retVal;
}

static int32_t test_mmcsd_file_io(char *fileName, char* fileData)
{
    int32_t retVal = SystemP_SUCCESS;
    uint32_t i;
    char buf[100];
    FF_FILE *testFp;
    
    /* Create file */
    testFp = ff_fopen(fileName, "w+");

    /* Write file data `TEST_MMCSD_FILE_LINE_CNT` times */

    uint32_t fileDataLen = strlen(fileData)+1;

    for(i = 0U; i < TEST_MMCSD_FILE_LINE_CNT; i++)
    {
        ff_fwrite(fileData, fileDataLen, 1, testFp);
    }

    /* Close file */
    ff_fclose(testFp);

    /* Re-open now for reading */
    testFp = ff_fopen(fileName, "r");

    /* Now read the lines one by one and check with fileData */
    for(i = 0U; i < TEST_MMCSD_FILE_LINE_CNT; i++)
    {
        ff_fread(buf, fileDataLen, 1, testFp);
        retVal |= strncmp(fileData, buf, fileDataLen);
        if(retVal != 0U)
        {
            break;
        }
    }

    /* Close file */
    ff_fclose(testFp);

    /* Delete file */
    ff_remove(fileName);

    return retVal;
}