/*
 * Copyright (C) 2022-23 Texas Instruments Incorporated
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
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/crypto/dthe/dthe.h>
#include <security/crypto/dthe/dthe_aes.h>
#include <kernel/dpl/ClockP.h>

/* Supported Operations */
#define APP_OPERATION_ENCRYPT           (1U)
#define APP_OPERATION_DECRYPT           (2U)

/* Supported Key length*/
#define APP_CRYPTO_AES_ECB_128          (128U)
#define APP_CRYPTO_AES_ECB_256          (256U)

/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_128_MAXKEY_LENGTH          (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH         (32U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_32K_BUF_LEN              (32768U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_16K_BUF_LEN              (16384U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_8K_BUF_LEN               (8192U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_4K_BUF_LEN               (4096U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_2K_BUF_LEN               (2048U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_1K_BUF_LEN               (1024U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_512B_BUF_LEN             (512U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                        (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                    (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                    (0xCE005000U)
/* Total number of Test case*/
#define TEST_CRYPTO_AES_TEST_CASES_COUNT              (28U)

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesEcb128Key[APP_CRYPTO_AES_ECB_128_MAXKEY_LENGTH] =
{
    0x93, 0x28, 0x67, 0x64, 0xA8, 0x51, 0x46, 0x73,
    0x0E, 0x64, 0x18, 0x88, 0xDB, 0x34, 0xEB, 0x47
};

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesEcb256Key[APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* input buffer for encryption or decryption */
uint8_t     gCryptoAesEcbInputBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Encryption output buf */
uint8_t     gCryptoAesEcbEncResultBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Decryption output buf */
uint8_t     gCryptoAesEcbDecResultBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));

typedef struct
{
    uint16_t key;
    char operation[20];
    uint16_t dataSize;
    double performance;
}App_benchmark;

/* Local test functions */
static void test_aes_ecb128_32kBuf(void *args);
static void test_aes_ecb128_16kBuf(void *args);
static void test_aes_ecb128_8kBuf(void *args);
static void test_aes_ecb128_4kBuf(void *args);
static void test_aes_ecb128_2kBuf(void *args);
static void test_aes_ecb128_1kBuf(void *args);
static void test_aes_ecb128_512bBuf(void *args);

static void test_aes_ecb256_32kBuf(void *args);
static void test_aes_ecb256_16kBuf(void *args);
static void test_aes_ecb256_8kBuf(void *args);
static void test_aes_ecb256_4kBuf(void *args);
static void test_aes_ecb256_2kBuf(void *args);
static void test_aes_ecb256_1kBuf(void *args);
static void test_aes_ecb256_512bBuf(void *args);
void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation);
static const char *bytesToString(uint64_t bytes);
void App_printPerformanceLogs(void);

DTHE_Handle         aesHandle;

uint16_t gCount = 0;
App_benchmark results[TEST_CRYPTO_AES_TEST_CASES_COUNT];

void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    DTHE_AES_Return_t             status;

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] AES ECB-128 Hw tests started ...\r\n");
    RUN_TEST(test_aes_ecb128_32kBuf,  8472, NULL);
    RUN_TEST(test_aes_ecb128_16kBuf,  8472, NULL);
    RUN_TEST(test_aes_ecb128_8kBuf,   8472, NULL);
    RUN_TEST(test_aes_ecb128_4kBuf,   8472, NULL);
    RUN_TEST(test_aes_ecb128_2kBuf,   8472, NULL);
    RUN_TEST(test_aes_ecb128_1kBuf,   8472, NULL);
    RUN_TEST(test_aes_ecb128_512bBuf, 8472, NULL);

    DebugP_log("[CRYPTO] AES ECB-256 Hw tests started ...\r\n");
    RUN_TEST(test_aes_ecb256_32kBuf,  8473, NULL);
    RUN_TEST(test_aes_ecb256_16kBuf,  8473, NULL);
    RUN_TEST(test_aes_ecb256_8kBuf,   8473, NULL);
    RUN_TEST(test_aes_ecb256_4kBuf,   8473, NULL);
    RUN_TEST(test_aes_ecb256_2kBuf,   8473, NULL);
    RUN_TEST(test_aes_ecb256_1kBuf,   8473, NULL);
    RUN_TEST(test_aes_ecb256_512bBuf, 8473, NULL);

    App_printPerformanceLogs();
    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(aesHandle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

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

void test_aes_ecb128_32kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_32K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();
    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_32K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_32K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_32K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb128_16kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_16K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_16K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_16K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);


    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_16K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb128_8kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_8K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_8K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_8K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_8K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb128_4kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_4K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_4K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_4K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);


    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_4K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb128_2kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_2K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_2K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_2K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);


    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_2K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb128_1kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_1K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_1K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_1K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);


    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_1K_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb128_512bBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_512B_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_512B_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb128Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_512B_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_512B_BUF_LEN, APP_CRYPTO_AES_ECB_128, APP_OPERATION_DECRYPT);


    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb256_32kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_32K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_32K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_32K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_32K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_32K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb256_16kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_16K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_16K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_16K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_16K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_16K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

}

void test_aes_ecb256_8kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_8K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_8K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_8K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_8K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_8K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb256_4kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_4K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_4K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_4K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_4K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_4K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb256_2kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_2K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_2K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_2K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_2K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_2K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb256_1kBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_1K_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_1K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_1K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_1K_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_1K_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_ecb256_512bBuf(void *args)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            t1, t2;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbInputBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_512B_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_512B_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_ENCRYPT);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcbDecResultBuf[0];
    aesParams.dataLenBytes      = TEST_CRYPTO_AES_TEST_512B_BUF_LEN;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcbEncResultBuf[0];

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_TEST_512B_BUF_LEN, APP_CRYPTO_AES_ECB_256, APP_OPERATION_DECRYPT);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

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

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation)
{
    uint32_t diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);
    throughputInMBps  = (numBytes * cpuClkMHz)/diffCnt;

    App_benchmark *table = &results[gCount++];
    table-> key = key;
    if(operation == APP_OPERATION_ENCRYPT)
    {
        strcpy(table->operation, "Encryption");
    }
    else
    {
        strcpy(table->operation, "Decryption");
    }
    table->dataSize = numBytes;
    table->performance = (double)(8 * throughputInMBps);
}

void App_printPerformanceLogs()
{
    double cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    DebugP_log("BENCHMARK START - SA2UL - AES - ECB \r\n");
    DebugP_log("- Software/Application used : test_dthe_aes_ecb \r\n");
    DebugP_log("- Code Placement            : OCRAM \r\n");
    DebugP_log("- Data Placement            : OCRAM \r\n");
    DebugP_log("- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB\r\n");
    DebugP_log("- CPU with operating speed  : R5F with %dMHZ \r\n", (uint32_t)cpuClkMHz);
    DebugP_log("| Key Length | operation  | Size | Performance (Mbps) | \r\n");
    DebugP_log("|-------------|------------|------|-------------| \r\n");
    for( uint32_t i = 0; i < TEST_CRYPTO_AES_TEST_CASES_COUNT; i++)
    {
        DebugP_log("| %d | %s | %s | %lf |\r\n", results[i].key,  \
                    results[i].operation, bytesToString(results[i].dataSize), results[i].performance);
    }
    DebugP_log("BENCHMARK END\r\n");
}

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
    },
};

uint32_t gDtheConfigNum = 1;