/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
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

/* This test demonstrates the HW implementation of SHA */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_sha.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_SHA_512                             (512U)
#define APP_SHA_256                             (256U)

#define TEST_CRYPTO_SHA_TEST_CASES_COUNT        (14U)
/* SHA512 length */
#define TEST_SHA512_LENGTH                      (64U)
/* SHA256 length */
#define TEST_SHA256_LENGTH                      (32U)
/* Sha 32k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_32K_BUF_LEN     (32768U)
/* Sha 32k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_16K_BUF_LEN     (16384U)
/* Sha 32k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_8K_BUF_LEN      (8192U)
/* Sha 32k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_4K_BUF_LEN      (4096U)
/* Sha 2k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_2K_BUF_LEN      (2048U)
/* Sha 1k buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_1K_BUF_LEN      (1024U)
/* Sha 512 bytes buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_512B_BUF_LEN    (512U)
/* Sha 256 bytes buf length*/
#define TEST_CRYPTO_SHA_HW_TEST_256B_BUF_LEN    (256U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                  (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE              (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE              (0xCE005000U)

/* number of tests*/
#define TEST_COUNT                              (14U)
/* number of test vectors*/
#define TEST_NUM_VECTORS                        (7U)

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES               (1U)

typedef struct
{
    uint16_t hashLength;
    uint16_t dataSize;
    double performance;
}App_benchmark;

typedef struct testParams_t
{
    uint32_t testInputLength;
    uint32_t testId;
    uint32_t hashAlgo;
    uint32_t hashLength;
    uint32_t keyLenForBenchMark;
    uint8_t *ptrExpectedOutput;
}testParams;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** 2k Test output buffer for sha computation  */
uint8_t gCryptoShaTestInputBuf[TEST_CRYPTO_SHA_HW_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/** 2k Test output buffer for sha computation  */
uint8_t gCryptoShaTestOutputBuf[TEST_CRYPTO_SHA_HW_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));

/** SHA-512 test vectors,this is expected hash for the gCryptoShaTestInputBuf buffer */
uint8_t gCryptoShaHw512_TestSums[TEST_NUM_VECTORS][TEST_SHA512_LENGTH] =
{
    {
        0x9b,0xd0,0xf5,0xf4,0xe6,0x00,0xca,0xcf,
        0xa4,0xef,0x0d,0x1d,0x4f,0x81,0x58,0x0d,
        0xc3,0x49,0xcb,0x5b,0x1a,0xd2,0xcd,0x7b,
        0x36,0x62,0xc3,0xbd,0x9e,0x2e,0xde,0x85,
        0x92,0xaf,0xc7,0xda,0xe9,0x86,0xd5,0x68,
        0x47,0xa0,0xb9,0x4e,0xb0,0xc9,0x6a,0x99,
        0x52,0xff,0x9c,0x2f,0x68,0x94,0x5c,0x09,
        0x96,0x83,0x70,0xfd,0x87,0x02,0x73,0xc4
    },
    {
        0xb5,0x5c,0x66,0x2a,0xdf,0x28,0x84,0x67,
        0x1d,0xe7,0x60,0x56,0x7d,0x97,0x5b,0xab,
        0xa8,0x3b,0xa7,0x87,0x46,0x51,0xe4,0x79,
        0x25,0xfd,0xc6,0x3b,0x4e,0x33,0x66,0xcb,
        0x6e,0x6d,0x3c,0x6a,0x46,0x5c,0x0a,0x8d,
        0x05,0xf2,0x1c,0x5b,0x03,0xcf,0x18,0xd6,
        0x41,0x96,0xec,0x4c,0xa5,0x74,0xbc,0xf5,
        0x79,0x01,0x77,0x91,0x81,0x1b,0xbd,0x64

    },
    {
        0x3c,0xd8,0x34,0x01,0xde,0x94,0x99,0x67,
        0x34,0x36,0xde,0x9f,0x23,0xd9,0x44,0x04,
        0x51,0xb1,0x65,0x8c,0x00,0x65,0xd4,0xf0,
        0x40,0x80,0x97,0x4d,0x67,0xc3,0xdf,0x34,
        0x78,0x69,0x5f,0xe6,0x2c,0xda,0xc3,0x6d,
        0x11,0x03,0xd3,0xab,0xcf,0x52,0xa5,0xb1,
        0x3b,0x58,0x34,0x9c,0x37,0xb5,0x63,0x0a,
        0xa2,0xc9,0x89,0x08,0x23,0xe4,0x9c,0x9c
    },
    {
        0xeb,0x70,0x40,0x94,0x8a,0x18,0x9a,0x59,
        0xd7,0x2d,0x1e,0x53,0x86,0x9f,0xba,0x1a,
        0xea,0xcb,0x6c,0x3b,0xe3,0x3c,0x7b,0xe5,
        0xd1,0xf0,0x3f,0x31,0xa9,0x66,0x00,0x33,
        0xb2,0x01,0x86,0x49,0xb3,0x33,0x25,0xb4,
        0x8b,0x31,0x79,0x44,0x66,0x4d,0x8e,0x71,
        0xa6,0x4a,0x7c,0x6f,0x29,0xdd,0x18,0xac,
        0xf1,0x62,0xc8,0xb0,0xd1,0x3a,0x21,0x4e
    },
    {
        0x15,0x65,0x12,0xd7,0x3a,0x85,0xd7,0xa4,
        0x25,0x6d,0xa5,0x7c,0x63,0xfd,0x8c,0x33,
        0x9e,0xd9,0x2d,0xa6,0xfa,0x33,0x62,0x0c,
        0xe1,0x55,0xa5,0x77,0x19,0x87,0xec,0x61,
        0xa6,0xa2,0xc7,0xd1,0x08,0x71,0x80,0x6b,
        0x30,0x25,0x8a,0xd0,0x15,0xd2,0xca,0x34,
        0x85,0x49,0xde,0x65,0xd9,0x68,0xaa,0xfe,
        0xae,0x9d,0xda,0x2b,0xf6,0x6a,0x21,0x05
    },
    {
        0x74,0xb2,0x24,0x92,0xe3,0xb9,0xa8,0x6a,
        0x9c,0x93,0xc2,0x3a,0x69,0xf8,0x21,0xeb,
        0xaf,0xa4,0x29,0x30,0x2c,0x1f,0x40,0x54,
        0xb4,0xbc,0x37,0x35,0x6a,0x4b,0xae,0x05,
        0x6d,0x9c,0xcb,0xc6,0xf2,0x40,0x93,0xa2,
        0x57,0x04,0xfa,0xaa,0x72,0xbd,0x21,0xa5,
        0xf3,0x37,0xca,0x9e,0xc9,0x2f,0x32,0x36,
        0x9d,0x24,0xe6,0xb9,0xfa,0xe9,0x54,0xd8
    },
    {
        0x02,0x10,0xd2,0x7b,0xcb,0xe0,0x5c,0x21,
        0x56,0x62,0x7c,0x5f,0x13,0x6a,0xde,0x13,
        0x38,0xab,0x98,0xe0,0x6a,0x45,0x91,0xa0,
        0x0b,0x0b,0xca,0xa6,0x16,0x62,0xa5,0x93,
        0x1d,0x0b,0x3b,0xd4,0x1a,0x67,0xb5,0xc1,
        0x40,0x62,0x79,0x23,0xf5,0xf6,0x30,0x76,
        0x69,0xeb,0x50,0x8d,0x8d,0xb3,0x8b,0x2a,
        0x8c,0xd4,0x1a,0xeb,0xd7,0x83,0x39,0x4b
    },
};

/** SHA-256 test vectors,this is expected hash for the gCryptoShaTestInputBuf[10] buffer*/
uint8_t gCryptoShaHw256_TestSums[TEST_NUM_VECTORS][TEST_SHA256_LENGTH] =
{
    {
        0xB2,0x17,0xB6,0x5E,0x6F,0x20,0x5F,0x41,
        0xB3,0xFB,0x8E,0xF9,0x0C,0xF7,0xC4,0x4D,
        0xA9,0x3F,0x63,0x0C,0xA0,0x39,0x65,0x27,
        0x34,0x85,0xBB,0xB2,0x1A,0x5C,0xCC,0xF5
    },
    {
        0xF3,0x33,0x6B,0xEA,0x75,0x2B,0x5A,0x28,
        0x74,0x30,0x33,0xDD,0x2C,0x84,0x4A,0x4A,
        0x63,0xFB,0xA0,0x88,0x71,0xAA,0xEE,0x25,
        0x86,0xA2,0xBF,0x2D,0x69,0xBE,0x83,0xA2
    },
    {
        0xDD,0x4E,0x67,0x30,0x52,0x09,0x32,0x76,
        0x7E,0xC0,0xA9,0xE3,0x3F,0xE1,0x9C,0x4C,
        0xE2,0x43,0x99,0xD6,0xEB,0xA4,0xFF,0x62,
        0xF1,0x30,0x13,0xC9,0xED,0x30,0xEF,0x87
    },
    {
        0xC9,0x3E,0xEE,0x2D,0x0D,0xB0,0x2F,0x10,
        0xAC,0xC7,0x46,0x0D,0x95,0x76,0xE1,0x22,
        0xDC,0xF8,0xCD,0x53,0xC4,0xBF,0x8D,0xFC,
        0xAE,0x1B,0x3E,0x74,0xEB,0xCF,0xFF,0x5A
    },
    {
        0xb2,0xa3,0xa5,0x02,0xfd,0xfc,0x34,0xf4,
        0xe3,0xed,0xfa,0x94,0xb7,0xf3,0x10,0x9c,
        0xd9,0x72,0xd8,0x7a,0x4f,0xec,0x63,0xab,
        0x21,0xa6,0x67,0x33,0x79,0xcc,0xf7,0xad
    },
    {
        0x2e,0xdc,0x98,0x68,0x47,0xe2,0x09,0xb4,
        0x01,0x6e,0x14,0x1a,0x6d,0xc8,0x71,0x6d,
        0x32,0x07,0x35,0x0f,0x41,0x69,0x69,0x38,
        0x2d,0x43,0x15,0x39,0xbf,0x29,0x2e,0x4a
    },
    {
        0x47,0x1b,0xe6,0x55,0x8b,0x66,0x5e,0x4f,
        0x6d,0xd4,0x9f,0x11,0x84,0x81,0x4d,0x14,
        0x91,0xb0,0x31,0x5d,0x46,0x6b,0xee,0xa7,
        0x68,0xc1,0x53,0xcc,0x55,0x00,0xc8,0x36
    },
};

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

uint16_t gCount = 0;
App_benchmark results[TEST_CRYPTO_SHA_TEST_CASES_COUNT];

/* Public context crypto dthe, aes and sha accelerators base address */
DTHE_Attrs gDTHE_Attrs[1] =
{
    {
        /* crypto accelerator base address */
        .caBaseAddr         = CSL_DTHE_PUBLIC_U_BASE,
        /* AES base address */
        .aesBaseAddr        = CSL_DTHE_PUBLIC_AES_U_BASE,
        /* SHA base address */
        .shaBaseAddr        = CSL_DTHE_PUBLIC_SHA_U_BASE,
        /* For checking dthe driver open or close */
        .isOpen             = FALSE,
    },
};

DTHE_Config gDtheConfig[1]=
{
    {
        &gDTHE_Attrs[0],
        DMA_DISABLE,
    },
};

uint32_t gDtheConfigNum = 1;

DMA_Config gDmaConfig[1]=
{
    {
        &gEdmaHandle[0],
        &gEdmaFxns,
    },
};
uint32_t gDmaConfigNum = 1;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Local test functions */
static void test_sha(void *args);
static void test_get_buf(uint8_t * buf, uint32_t sizeInBytes);
void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t hash);
static const char *bytesToString(uint64_t bytes);
void App_printPerformanceLogs(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    uint32_t loopForTestCount = 0;

    testParams testRunParams[] = {
        {TEST_CRYPTO_SHA_HW_TEST_32K_BUF_LEN, 96,   DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[0]},
        {TEST_CRYPTO_SHA_HW_TEST_16K_BUF_LEN, 1203, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[1]},
        {TEST_CRYPTO_SHA_HW_TEST_8K_BUF_LEN,  1204, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[2]},
        {TEST_CRYPTO_SHA_HW_TEST_4K_BUF_LEN,  1205, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[3]},
        {TEST_CRYPTO_SHA_HW_TEST_2K_BUF_LEN,  1206, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[4]},
        {TEST_CRYPTO_SHA_HW_TEST_1K_BUF_LEN,  1207, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[5]},
        {TEST_CRYPTO_SHA_HW_TEST_512B_BUF_LEN,1208, DTHE_SHA_ALGO_SHA512, TEST_SHA512_LENGTH, APP_SHA_512, gCryptoShaHw512_TestSums[6]},
        {TEST_CRYPTO_SHA_HW_TEST_32K_BUF_LEN, 97,   DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[0]},
        {TEST_CRYPTO_SHA_HW_TEST_16K_BUF_LEN, 1209, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[1]},
        {TEST_CRYPTO_SHA_HW_TEST_8K_BUF_LEN,  1210, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[2]},
        {TEST_CRYPTO_SHA_HW_TEST_4K_BUF_LEN,  1211, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[3]},
        {TEST_CRYPTO_SHA_HW_TEST_2K_BUF_LEN,  1212, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[4]},
        {TEST_CRYPTO_SHA_HW_TEST_1K_BUF_LEN,  1213, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[5]},
        {TEST_CRYPTO_SHA_HW_TEST_512B_BUF_LEN,1214, DTHE_SHA_ALGO_SHA256, TEST_SHA256_LENGTH, APP_SHA_256, gCryptoShaHw256_TestSums[6]},
    };

    for (loopForTestCount = 0; loopForTestCount < TEST_COUNT; loopForTestCount++)
    {
        RUN_TEST(test_sha, (uint32_t)testRunParams[loopForTestCount].testId, (void *)&testRunParams[loopForTestCount]);
    }

    App_printPerformanceLogs();
    UNITY_END();
    Board_driversClose();
    Drivers_close();
}

/** Unity framework required functions */
void setUp(void)
{
}

void tearDown(void)
{
}

void test_sha(void *args)
{
    DTHE_SHA_Return_t   status;
    DTHE_Handle         shaHandle;
    DTHE_SHA_Params     shaParams;
    uint32_t            t1, t2;

    testParams *ptrTestPrms =  args;

    /* Opening crypto driver */
    shaHandle = DTHE_open(0);
    DebugP_assert(shaHandle != NULL);

    test_get_buf(gCryptoShaTestInputBuf, ptrTestPrms->testInputLength);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Opening sha driver */
    status = DTHE_SHA_open(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Initialize the SHA Parameters */
    shaParams.algoType          = ptrTestPrms->hashAlgo;
    shaParams.ptrDataBuffer     = (uint32_t*)&gCryptoShaTestInputBuf[0];
    shaParams.dataLenBytes      = ptrTestPrms->testInputLength;

    /* Performing DTHE SHA operation */
    status = DTHE_SHA_compute(shaHandle, &shaParams, TRUE);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Closing sha driver */
    status = DTHE_SHA_close(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, ptrTestPrms->testInputLength, ptrTestPrms->keyLenForBenchMark);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(shaHandle))
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_SHA_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    if(memcmp(shaParams.digest, ptrTestPrms->ptrExpectedOutput, ptrTestPrms->hashLength) != 0)
    {
        TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, 0);
    }
}

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t hash)
{
    double diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);
    throughputInMBps  = (numBytes * cpuClkMHz)/diffCnt;

    App_benchmark *table = &results[gCount++];
    table->hashLength = hash;
    table->dataSize = numBytes;
    table->performance = (double)(8 * throughputInMBps);
}

void test_get_buf(uint8_t * buf, uint32_t sizeInBytes)
{
    memset(buf,0x61,sizeInBytes);
    /* Perform cache writeback */
    CacheP_wb(buf, sizeInBytes, CacheP_TYPE_ALLD);
    /* Perform cache writeback */
    CacheP_inv(buf, sizeInBytes, CacheP_TYPE_ALLD);
}

static const char *bytesToString(uint64_t bytes)
{
	char *suffix[] = {"B", "KB", "MB", "GB", "TB"};
	char length = sizeof(suffix) / sizeof(suffix[0]);

	int i = 0;
	double dblBytes = bytes;

	if (bytes > 1024) {
		for (i = 0; (bytes / 1024) > 0 && i<length-1; i++, bytes /= 1024)
			dblBytes = bytes / 1024.0;
	}

	static char output[200];
	sprintf(output, "%  .02lf %s", dblBytes, suffix[i]);
	return output;
}

void App_printPerformanceLogs()
{
    double cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    DebugP_log("BENCHMARK START - DTHE - SHA \r\n");
    DebugP_log("- Software/Application used : test_dthe_sha \r\n");
    DebugP_log("- Code Placement            : OCMC \r\n");
    DebugP_log("- Data Placement            : OCMC \r\n");
    DebugP_log("- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB\r\n");
    DebugP_log("- CPU with operating speed  : R5F with %dMHZ \r\n", (uint32_t)cpuClkMHz);
    DebugP_log("| SHA | Size | Performance (Mbps) | \r\n");
    DebugP_log("|-----|------|-------------| \r\n");
    for( uint32_t i = 0; i < TEST_CRYPTO_SHA_TEST_CASES_COUNT; i++)
    {
        DebugP_log("| %d | %s | %lf |\r\n", results[i].hashLength, bytesToString(results[i].dataSize), results[i].performance);
    }
    DebugP_log("BENCHMARK END\r\n");
}
