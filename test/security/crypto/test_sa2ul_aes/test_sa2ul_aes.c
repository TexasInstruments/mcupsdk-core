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

/* This test demonstrates the HW implementation of SHA */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* input or output length*/
#define APP_CRYPTO_AES_CBC_128_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_CBC_128_MAXKEY_LENGTH          (16U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_CBC_128_MAXIV_LENGTH           (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_CBC_128_KEY_LENGTH_IN_BITS     (128U)
/* Aes max key length*/
#define APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH         (32U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_CBC_256_KEY_LENGTH_IN_BITS     (256U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN           (32768U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN           (16384U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN            (8192U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN            (4096U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN            (2048U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN            (1024U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN          (512U)

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesCbc128Key[APP_CRYPTO_AES_CBC_128_MAXKEY_LENGTH] =
{
    0x93, 0x28, 0x67, 0x64, 0xA8, 0x51, 0x46, 0x73,
    0x0E, 0x64, 0x18, 0x88, 0xDB, 0x34, 0xEB, 0x47
};

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesCbc256Key[APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCbcIv[APP_CRYPTO_AES_CBC_128_MAXIV_LENGTH] =
{
    0x19, 0x2D, 0x9B, 0x3A, 0xA1, 0x0B, 0xB2, 0xF7,
    0x84, 0x6C, 0xCB, 0xA0, 0x08, 0x5C, 0x65, 0x7A
};

/* input buffer for encryption or decryption */
uint8_t     gCryptoAesCbcInputBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));
/* Encryption output buf */
uint8_t     gCryptoAesCbcEncResultBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));
/* Decryption output buf */
uint8_t     gCryptoAesCbcDecResultBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));

/* Context memory */
static Crypto_Context gCryptoAesCbcContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));


/* Local test functions */
static void test_aes_cbc128_32kBuf(void *args);
static void test_aes_cbc128_16kBuf(void *args);
static void test_aes_cbc128_8kBuf(void *args);
static void test_aes_cbc128_4kBuf(void *args);
static void test_aes_cbc128_2kBuf(void *args);
static void test_aes_cbc128_1kBuf(void *args);
static void test_aes_cbc128_512bBuf(void *args);

static void test_aes_cbc256_32kBuf(void *args);
static void test_aes_cbc256_16kBuf(void *args);
static void test_aes_cbc256_8kBuf(void *args);
static void test_aes_cbc256_4kBuf(void *args);
static void test_aes_cbc256_2kBuf(void *args);
static void test_aes_cbc256_1kBuf(void *args);
static void test_aes_cbc256_512bBuf(void *args);
static void test_get_buf(uint8_t * buf, uint32_t sizeInBytes);
void App_printPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes);

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    int32_t             status;
    Crypto_Handle    aesHandle;

    aesHandle = Crypto_open(&gCryptoAesCbcContext);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] AES CBC-128 Hw tests started ...\r\n");
    RUN_TEST(test_aes_cbc128_32kBuf,  2303, NULL);
    RUN_TEST(test_aes_cbc128_16kBuf,  2304, NULL);
    RUN_TEST(test_aes_cbc128_8kBuf,   2305, NULL);
    RUN_TEST(test_aes_cbc128_4kBuf,   2306, NULL);
    RUN_TEST(test_aes_cbc128_2kBuf,   2307, NULL);
    RUN_TEST(test_aes_cbc128_1kBuf,   2308, NULL);
    RUN_TEST(test_aes_cbc128_512bBuf, 2309, NULL);

    DebugP_log("[CRYPTO] AES CBC-256 Hw tests started ...\r\n");
    RUN_TEST(test_aes_cbc256_32kBuf,  2310, NULL);
    RUN_TEST(test_aes_cbc256_16kBuf,  2311, NULL);
    RUN_TEST(test_aes_cbc256_8kBuf,   2312, NULL);
    RUN_TEST(test_aes_cbc256_4kBuf,   2313, NULL);
    RUN_TEST(test_aes_cbc256_2kBuf,   2314, NULL);
    RUN_TEST(test_aes_cbc256_1kBuf,   2315, NULL);
    RUN_TEST(test_aes_cbc256_512bBuf, 2316, NULL);
    
    /* Close AES instance */
    status = Crypto_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

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

void test_aes_cbc128_32kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_16kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_8kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_4kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_2kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_1kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc128_512bBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;

    memset(gCryptoAesCbcInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);


    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_32kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_16kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_8kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_4kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_2kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_1kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_cbc256_512bBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbcIv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcInputBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesCbcEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Encryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbcEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesCbcDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    DebugP_log("Decryption Performance :\r\n");
    App_printPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    /* comparing result with expected test results */
    status = memcmp(gCryptoAesCbcDecResultBuf, gCryptoAesCbcInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void App_printPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes)
{
    uint32_t diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);

    DebugP_log("[CRYPTO] Tick-1 : %ld  \r\n", (uint64_t)t1);
    DebugP_log("[CRYPTO] Tick-2 : %ld  \r\n", (uint64_t)t2);
    DebugP_log("[CRYPTO] Data length : %d bytes \r\n", numBytes);
    DebugP_log("[CRYPTO] Total ticks : %ld \r\n", (uint64_t)(diffCnt));

    throughputInMBps  = (numBytes * cpuClkMHz)/diffCnt;
    
    DebugP_log("[CRYPTO] Total throughput In Mbps  : %lf \r\n", (double)(8 * throughputInMBps));
}