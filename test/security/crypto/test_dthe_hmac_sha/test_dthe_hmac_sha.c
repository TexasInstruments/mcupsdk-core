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

/* This test demonstrates the HW implementation of HMAC SHA */

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

#define APP_HMAC_SHA_512                             (512U)
#define APP_HMAC_SHA_256                             (256U)

#define TEST_CRYPTO_AES_TEST_CASES_COUNT        (14U)
/* HMAC SHA-256 length */
#define TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH                   (32U)
/* HMAC SHA-512 length */
#define TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH                    (64U)
/* Alignment */
#define TEST_CRYPTO_HMAC_SHA_BUF_ALIGNMENT                      (128U)
/* HMAC SHA-256 key length */
#define TEST_CRYPTO_HMAC_SHA_KEYLEN_BYTES                       (64U)
/* 32k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_32K_BUF_LEN                   (32768U)
/* 16k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_16K_BUF_LEN                   (16384U)
/* 8k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_8K_BUF_LEN                    (8192U)
/* 4k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_4K_BUF_LEN                    (4096U)
/* 2k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_2K_BUF_LEN                    (2048U)
/* 1k buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_1K_BUF_LEN                    (1024U)
/* 512 bytes buf length*/
#define TEST_CRYPTO_HMAC_SHA_TEST_512B_BUF_LEN                  (512U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                                  (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                              (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                              (0xCE005000U)

/* number of tests*/
#define TEST_COUNT                                              (14U)
/* number of test vectors*/
#define TEST_NUM_VECTORS                                        (7U)

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                     (1U)

typedef struct testParams_t
{
    uint32_t testInputLength;
    uint32_t testId;
    uint32_t hashAlgo;
    uint32_t hashLength;
    uint32_t keyLenForBenchMark;
    uint8_t *ptrExpectedOutput;
}testParams;

typedef struct
{
    uint16_t hashLength;
    uint16_t dataSize;
    double performance;
}App_benchmark;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** 32k Test input buffer for HMAC sha computation  */
uint8_t gCryptoShaTestInputBuf[TEST_CRYPTO_HMAC_SHA_TEST_32K_BUF_LEN] __attribute__((aligned(TEST_CRYPTO_HMAC_SHA_BUF_ALIGNMENT), section(".bss.filebuf")));

/** HMAC SHA-512 test vectors,this is expected hash for the gCryptoShaTestInputBuf buffer */
uint8_t gCryptoHmacShaHw512_TestSums[TEST_NUM_VECTORS][TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH] =
{
    {
        0x1d, 0xd1, 0x50, 0x35, 0x31, 0xab, 0x4d, 0xab, 
        0xc2, 0x53, 0xca, 0xdf, 0x6c, 0xfe, 0xaf, 0x69, 
        0x9b, 0x11, 0xd1, 0x3f, 0x0b, 0x36, 0x0c, 0x6b, 
        0x4a, 0x6a, 0x8e, 0xa3, 0x7a, 0x40, 0xe9, 0x46, 
        0x93, 0x27, 0x69, 0xdb, 0xdd, 0xd9, 0x07, 0x3a, 
        0xda, 0x78, 0x59, 0x61, 0x59, 0x4d, 0xc1, 0xd1, 
        0xa4, 0x66, 0xfb, 0x4b, 0x96, 0x76, 0x92, 0x7b, 
        0xf7, 0xf7, 0x56, 0x6f, 0x9d, 0x95, 0x00, 0x84
    },
    {
        0x6e, 0x51, 0x92, 0xb9, 0x1f, 0x53, 0x22, 0xe0, 
        0xb5, 0xa4, 0x5c, 0xa8, 0xec, 0xde, 0x2e, 0xd3, 
        0x89, 0x47, 0x1b, 0x05, 0x6e, 0xa8, 0x5f, 0xc1, 
        0x22, 0x51, 0x01, 0x78, 0x2d, 0x39, 0x01, 0xac, 
        0x8e, 0x6d, 0xd6, 0x6e, 0x70, 0x9d, 0xa2, 0xe1, 
        0x7e, 0x2f, 0x65, 0x6f, 0xd4, 0x90, 0xb2, 0x95, 
        0xec, 0xc3, 0x0d, 0x82, 0x4c, 0xe0, 0xcc, 0x81, 
        0x10, 0x3e, 0xee, 0xce, 0x56, 0x92, 0xc6, 0xf5
    },
    {
        0x28, 0xb3, 0xb9, 0x30, 0x4f, 0xc4, 0x1b, 0x4d, 
        0x8f, 0x6d, 0x87, 0x03, 0x6f, 0x6f, 0xb0, 0xff, 
        0x41, 0x94, 0x15, 0x73, 0x56, 0xdc, 0xfa, 0x3c, 
        0x62, 0x68, 0x56, 0x8a, 0x2f, 0x71, 0xb0, 0x14, 
        0x17, 0xe1, 0x76, 0xa2, 0x27, 0x7d, 0x1b, 0xc6, 
        0xcb, 0x85, 0xcc, 0xef, 0xaf, 0x39, 0x48, 0x1f, 
        0xd9, 0x2b, 0xf3, 0xc2, 0xb2, 0x91, 0xa0, 0x5a, 
        0x3a, 0xc6, 0xbd, 0x11, 0x9d, 0x9b, 0x5b, 0xa4
    },    
    {
        0xf2, 0xf5, 0xab, 0x54, 0x2f, 0x5b, 0x02, 0x7b, 
        0xd4, 0x7e, 0x31, 0x9d, 0x93, 0xc2, 0xf4, 0x28, 
        0x89, 0x96, 0x79, 0x79, 0x19, 0x80, 0x16, 0x8d, 
        0xc4, 0xb6, 0x76, 0x69, 0x39, 0x8a, 0x81, 0xbc, 
        0x79, 0xaf, 0x30, 0x54, 0x8c, 0xda, 0x5c, 0x48, 
        0xe0, 0x30, 0xb4, 0xcd, 0xc9, 0x51, 0xe6, 0x12, 
        0x88, 0x56, 0xab, 0xdc, 0x64, 0xd1, 0x26, 0xba, 
        0xfb, 0x66, 0xeb, 0x46, 0xa1, 0xbb, 0x92, 0xcc
    },
    {
        0xb8, 0x69, 0x32, 0x3f, 0x0f, 0x6c, 0x53, 0x10, 
        0x0d, 0x31, 0x7c, 0x2e, 0xa4, 0x63, 0x32, 0x96, 
        0x3e, 0x5b, 0xf8, 0x39, 0xfd, 0xb6, 0xcb, 0x86, 
        0x38, 0x6d, 0xd3, 0x3b, 0x41, 0x8f, 0x6a, 0x73, 
        0xc9, 0x6f, 0xee, 0x79, 0x5f, 0xab, 0xff, 0x5d, 
        0xc8, 0x53, 0x7e, 0x5d, 0x61, 0xe5, 0x9c, 0xe8, 
        0x4b, 0x8f, 0x08, 0xf3, 0x3b, 0x6e, 0x0f, 0xd0, 
        0x8f, 0x97, 0x9c, 0xd7, 0x14, 0x26, 0x34, 0x7b
    },
    {
        0x54, 0x27, 0xab, 0x51, 0xfd, 0x8d, 0x05, 0xac, 
        0x84, 0xe7, 0xc4, 0x25, 0xeb, 0x8c, 0xf4, 0xa3, 
        0x05, 0x19, 0xae, 0x06, 0x4c, 0x21, 0x98, 0x16, 
        0xf9, 0xd2, 0xc2, 0x55, 0x01, 0xd3, 0xe5, 0x1b, 
        0xb5, 0xfa, 0x9d, 0x66, 0x92, 0xfd, 0x6a, 0xaa, 
        0xf8, 0x83, 0xd8, 0xa8, 0x43, 0x79, 0x89, 0x55, 
        0xea, 0xad, 0x63, 0x5e, 0x84, 0x81, 0xb4, 0x20, 
        0xd4, 0x21, 0xd7, 0x08, 0x6f, 0x3c, 0xbd, 0x25
    },
    {
        0xbd, 0x81, 0x7a, 0x1e, 0xb7, 0x85, 0xc7, 0x55, 
        0x88, 0x60, 0x1c, 0xc7, 0xa6, 0xc1, 0x67, 0xa1, 
        0x8e, 0x6f, 0xb4, 0xcd, 0xd1, 0x2b, 0xb5, 0x46, 
        0xd4, 0x51, 0x60, 0xdd, 0x1f, 0xe2, 0x24, 0x18, 
        0xa8, 0x52, 0xaa, 0x18, 0xd1, 0xd4, 0xa9, 0x2f, 
        0xbe, 0x23, 0x79, 0x91, 0xfe, 0x68, 0xcf, 0xa3, 
        0x69, 0x31, 0x4f, 0x2c, 0x0e, 0x34, 0xb4, 0x99, 
        0xc2, 0xf0, 0x21, 0x9f, 0xe3, 0x09, 0x17, 0x66
    }
};

/** HMAC SHA-256 test vectors,this is expected hash for the gCryptoShaTestInputBuf[10] buffer*/
uint8_t gCryptoHmacSha256_TestSums[TEST_NUM_VECTORS][TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH] =
{
    {
        0xea, 0xd3, 0xb9, 0x4e, 0x2c, 0x6f, 0x78, 0x75, 
        0x7b, 0xcd, 0x61, 0xa8, 0xd1, 0x42, 0x1b, 0xb7, 
        0x18, 0x60, 0xee, 0x0c, 0x3f, 0x51, 0x23, 0x52, 
        0x6e, 0xb8, 0xb8, 0x8c, 0x56, 0xce, 0xdd, 0x00
    },
    {
        0x63, 0x7d, 0xe9, 0xc5, 0xc5, 0x2a, 0xbf, 0x07, 
        0xe1, 0xc1, 0xb6, 0x33, 0x6d, 0x5c, 0x71, 0xcf, 
        0x5f, 0x1c, 0xae, 0x30, 0x8a, 0x50, 0x9a, 0x2e, 
        0x92, 0x7d, 0xcc, 0x91, 0x22, 0x58, 0x2e, 0xd0
    },
    {
        0x5f, 0x36, 0x63, 0xb9, 0x7b, 0xd0, 0x88, 0x7d, 
        0x63, 0x1b, 0x81, 0xf3, 0x0a, 0xcf, 0x84, 0x46, 
        0xd4, 0x38, 0x59, 0x56, 0xde, 0x71, 0x46, 0x8e, 
        0xa9, 0x30, 0x02, 0x37, 0xef, 0x88, 0x71, 0x1e
    },
    {
        0x27, 0x64, 0x78, 0x84, 0xc3, 0x2e, 0x44, 0xde, 
        0xa4, 0xf3, 0xb4, 0x64, 0x33, 0x71, 0x80, 0xa4, 
        0xf9, 0xea, 0xcf, 0x42, 0xdd, 0x7f, 0xac, 0xea, 
        0x94, 0x28, 0x71, 0x57, 0x29, 0x0d, 0x9c, 0xd0
    },
    {
        0xc6, 0x7a, 0xf2, 0x04, 0x67, 0x7e, 0xb3, 0xcb, 
        0xbf, 0xa9, 0x45, 0xb2, 0x64, 0x40, 0xc9, 0x7f, 
        0x06, 0x76, 0xc9, 0x2f, 0x7f, 0x8a, 0x9b, 0xc1, 
        0x4b, 0x97, 0x8a, 0x38, 0x51, 0xf5, 0x01, 0x00
    },
    {
        0xe2, 0x36, 0x77, 0xb0, 0x92, 0xaa, 0xd2, 0x0c, 
        0x9c, 0x46, 0x9f, 0x03, 0x49, 0x9e, 0x6e, 0xfa, 
        0x8b, 0x78, 0x5b, 0x6c, 0xe9, 0x94, 0x40, 0xd8, 
        0xd1, 0xe9, 0x33, 0xb6, 0xcb, 0x96, 0xfe, 0x53
    },
    {
        0x1d, 0x1f, 0xdf, 0xc3, 0xb6, 0xc3, 0xa3, 0x35, 
        0x53, 0xfa, 0x6d, 0x68, 0xdd, 0x57, 0x0c, 0xb6, 
        0x59, 0x0e, 0x2f, 0x6f, 0x3c, 0xb8, 0x55, 0x33, 
        0x52, 0xe5, 0x0b, 0xda, 0xad, 0x66, 0x2a, 0x2e
    },
};

/* Key buffer for hmac computation */
static uint8_t gCryptoHmacShaKey[TEST_CRYPTO_HMAC_SHA_KEYLEN_BYTES] =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU,
    0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU,
    0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U, 0x27U, 0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2DU, 0x2EU, 0x2FU,
    0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U, 0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU
};

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

uint16_t gCount = 0;
App_benchmark results[TEST_CRYPTO_AES_TEST_CASES_COUNT];

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
static void test_hmac_sha(void *args);
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
        {TEST_CRYPTO_HMAC_SHA_TEST_32K_BUF_LEN, 8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[0]},
        {TEST_CRYPTO_HMAC_SHA_TEST_16K_BUF_LEN, 8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[1]},
        {TEST_CRYPTO_HMAC_SHA_TEST_8K_BUF_LEN,  8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[2]},
        {TEST_CRYPTO_HMAC_SHA_TEST_4K_BUF_LEN,  8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[3]},
        {TEST_CRYPTO_HMAC_SHA_TEST_2K_BUF_LEN,  8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[4]},
        {TEST_CRYPTO_HMAC_SHA_TEST_1K_BUF_LEN,  8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[5]},
        {TEST_CRYPTO_HMAC_SHA_TEST_512B_BUF_LEN,8589, DTHE_SHA_ALGO_SHA512, TEST_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, APP_HMAC_SHA_512, gCryptoHmacShaHw512_TestSums[6]},
        {TEST_CRYPTO_HMAC_SHA_TEST_32K_BUF_LEN, 8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[0]},
        {TEST_CRYPTO_HMAC_SHA_TEST_16K_BUF_LEN, 8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH,APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[1]},
        {TEST_CRYPTO_HMAC_SHA_TEST_8K_BUF_LEN,  8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[2]},
        {TEST_CRYPTO_HMAC_SHA_TEST_4K_BUF_LEN,  8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[3]},
        {TEST_CRYPTO_HMAC_SHA_TEST_2K_BUF_LEN,  8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[4]},
        {TEST_CRYPTO_HMAC_SHA_TEST_1K_BUF_LEN,  8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[5]},
        {TEST_CRYPTO_HMAC_SHA_TEST_512B_BUF_LEN,8590, DTHE_SHA_ALGO_SHA256, TEST_CRYPTO_HMAC_SHA256_OUTPUT_LENGTH, APP_HMAC_SHA_256, gCryptoHmacSha256_TestSums[6]},
    };

    for (loopForTestCount = 0; loopForTestCount < TEST_COUNT; loopForTestCount++)
    {
        /** Hmac sha512 and sha256 computations*/
        RUN_TEST(test_hmac_sha, (uint32_t)testRunParams[loopForTestCount].testId, (void *)&testRunParams[loopForTestCount]);        
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

void test_hmac_sha(void *args)
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
    shaParams.ptrKey            = (uint32_t*)&gCryptoHmacShaKey[0];
    shaParams.keySize           = TEST_CRYPTO_HMAC_SHA_KEYLEN_BYTES;

    /* Performing DTHE HMAC SHA operation */
    status = DTHE_HMACSHA_compute(shaHandle, &shaParams);
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

    status = memcmp(shaParams.digest, ptrTestPrms->ptrExpectedOutput, ptrTestPrms->hashLength);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
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
    DebugP_log("BENCHMARK START - DTHE - HMAC - SHA \r\n");
    DebugP_log("- Software/Application used : test_dthe_hmac_sha \r\n");
    DebugP_log("- Code Placement            : OCMC \r\n");
    DebugP_log("- Data Placement            : OCMC \r\n");
    DebugP_log("- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB\r\n");
    DebugP_log("- CPU with operating speed  : R5F with %dMHZ \r\n", (uint32_t)cpuClkMHz);
    DebugP_log("| SHA | Size | Performance (Mbps) | \r\n");
    DebugP_log("|-----|------|-------------| \r\n");
    for( uint32_t i = 0; i < TEST_CRYPTO_AES_TEST_CASES_COUNT; i++)
    {
        DebugP_log("| %d | %s | %lf |\r\n", results[i].hashLength, bytesToString(results[i].dataSize), results[i].performance);
    }
    DebugP_log("BENCHMARK END\r\n");
}
