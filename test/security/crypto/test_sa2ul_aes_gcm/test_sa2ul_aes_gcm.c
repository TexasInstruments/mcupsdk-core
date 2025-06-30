/*
 * Copyright (C) 2025 Texas Instruments Incorporated
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

/* Supported Operations */
#define APP_OPERATION_GHASH             (0U)
#define APP_OPERATION_ENCRYPT           (1U)
#define APP_OPERATION_DECRYPT           (2U)

/* Supported Key length*/
#define APP_CRYPTO_AES_GCM_128          (128U)
#define APP_CRYPTO_AES_GCM_256          (256U)

/* Aes max key length*/
#define APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH          (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_GCM_128_KEY_LENGTH_IN_BITS     (128U)
/* Aes max key length*/
#define APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH         (32U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_GCM_256_KEY_LENGTH_IN_BITS     (256U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_GCM_MAXIV_LENGTH               (12U)
/* Aes ghash zero array length */
#define APP_CRYPTO_AES_GCM_ZEROARRAY_LENGTH           (16U)
/* Aes max AAD length*/
#define APP_CRYPTO_AES_GCM_MAXAAD_LENGTH              (16U)
/* Aes Authentication tag length*/
#define APP_CRYPTO_AES_GCM_AUTHTAG_LENGTH             (16U)
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
/* Total number of Test case*/
#define TEST_CRYPTO_AES_TEST_CASES_COUNT              (42U)

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesGcm128Key[APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH] =
{
    0x93, 0x28, 0x67, 0x64, 0xA8, 0x51, 0x46, 0x73,
    0x0E, 0x64, 0x18, 0x88, 0xDB, 0x34, 0xEB, 0x47
};

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesGcm256Key[APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesGcmIv[APP_CRYPTO_AES_GCM_MAXIV_LENGTH] =
{
    0x19, 0x2D, 0x9B, 0x3A, 0xA1, 0x0B, 0xB2, 0xF7,
    0x84, 0x6C, 0xCB, 0xA0
};

/* Additional Authenticated Data (AAD) */
static uint8_t gCryptoAesGcmAad[APP_CRYPTO_AES_GCM_MAXAAD_LENGTH] = {
    0x24, 0x82, 0x56, 0x02, 0xbd, 0x12, 0xa9, 0x84,
    0xe0, 0x09, 0x2d, 0x3e, 0x44, 0x8e, 0xda, 0x5f
};

static uint8_t gZeroArray[APP_CRYPTO_AES_GCM_ZEROARRAY_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* input buffer for encryption or decryption */
uint8_t     gCryptoAesGcmInputBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));
/* Encryption output buf */
uint8_t     gCryptoAesGcmEncResultBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));
/* Decryption output buf */
uint8_t     gCryptoAesGcmDecResultBuf[TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN] __attribute__((aligned(SA2UL_CACHELINE_ALIGNMENT), section(".bss.filebuf")));

/* GHASH generated */
static uint8_t     gHash[SA2UL_GHASH_LENGTH_BYTES] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* AES GCM Authentication tag buffer */
static uint8_t     gCryptoAesGcmTagBuf[APP_CRYPTO_AES_GCM_MAXAAD_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
static Crypto_Context gCryptoAesGcmContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

typedef struct
{
    uint16_t key;
    char operation[20];
    uint16_t dataSize;
    double performance;
}App_benchmark;

/* Local test functions */
static void test_aes_gcm128_32kBuf(void *args);
static void test_aes_gcm128_16kBuf(void *args);
static void test_aes_gcm128_8kBuf(void *args);
static void test_aes_gcm128_4kBuf(void *args);
static void test_aes_gcm128_2kBuf(void *args);
static void test_aes_gcm128_1kBuf(void *args);
static void test_aes_gcm128_512bBuf(void *args);

static void test_aes_gcm256_32kBuf(void *args);
static void test_aes_gcm256_16kBuf(void *args);
static void test_aes_gcm256_8kBuf(void *args);
static void test_aes_gcm256_4kBuf(void *args);
static void test_aes_gcm256_2kBuf(void *args);
static void test_aes_gcm256_1kBuf(void *args);
static void test_aes_gcm256_512bBuf(void *args);
static void test_get_buf(uint8_t * buf, uint32_t sizeInBytes);
void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation);
static const char *bytesToString(uint64_t bytes);
void App_printPerformanceLogs(void);

uint16_t gCount = 0;
App_benchmark results[TEST_CRYPTO_AES_TEST_CASES_COUNT];

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    int32_t             status;
    Crypto_Handle    aesHandle;

    aesHandle = Crypto_open(&gCryptoAesGcmContext);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] AES GCM-128 Hw tests started ...\r\n");
    RUN_TEST(test_aes_gcm128_32kBuf,  14715, NULL);
    RUN_TEST(test_aes_gcm128_16kBuf,  14727, NULL);
    RUN_TEST(test_aes_gcm128_8kBuf,   14726, NULL);
    RUN_TEST(test_aes_gcm128_4kBuf,   14723, NULL);
    RUN_TEST(test_aes_gcm128_2kBuf,   14728, NULL);
    RUN_TEST(test_aes_gcm128_1kBuf,   14724, NULL);
    RUN_TEST(test_aes_gcm128_512bBuf, 14722, NULL);

    DebugP_log("[CRYPTO] AES GCM-256 Hw tests started ...\r\n");
    RUN_TEST(test_aes_gcm256_32kBuf,  14725, NULL);
    RUN_TEST(test_aes_gcm256_16kBuf,  14717, NULL);
    RUN_TEST(test_aes_gcm256_8kBuf,   14716, NULL);
    RUN_TEST(test_aes_gcm256_4kBuf,   14719, NULL);
    RUN_TEST(test_aes_gcm256_2kBuf,   14718, NULL);
    RUN_TEST(test_aes_gcm256_1kBuf,   14721, NULL);
    RUN_TEST(test_aes_gcm256_512bBuf, 14720, NULL);

    App_printPerformanceLogs();
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

void test_aes_gcm128_32kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gCryptoAesGcmInputBuf);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesGcmInputBuf);
    
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, sizeof(gCryptoAesGcmInputBuf), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, sizeof(gCryptoAesGcmInputBuf), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcmInputBuf[0], sizeof(gCryptoAesGcmInputBuf), gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcmEncResultBuf[0], sizeof(gCryptoAesGcmEncResultBuf), gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_16kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/ 
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_8kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_4kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_2kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_1kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Copy computed authetication tag result to authentication tag bufffer */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Compare decrypted data with original input data */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Compare the authentication tag generated by both encryption and decryption*/
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm128_512bBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    /* Copy 128-bit key */
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], SA2UL_MAX_KEY_SIZE_BYTES/2);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_128, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_32kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;  /* Using 256-bit key size */
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    /* Copy 256-bit key */
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_32K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_16kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;  /* Using 256-bit key size */
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    /* Copy 256-bit key */
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_16K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_8kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    /* Copy 256-bit key */
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen  = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy(&ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_8K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_4kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_4K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_2kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_2K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_1kBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_1K_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

void test_aes_gcm256_512bBuf(void *args)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;
    uint32_t            t1, t2;

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], SA2UL_MAX_KEY_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Initialize input buffer with zeros */
    (void) memset(gCryptoAesGcmInputBuf,0x00,TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);

    /* Start performance counter for GHASH operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Allocate context for GHASH generation */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for zero array */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Generate GHASH */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free GHASH context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record GHASH performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_GHASH);

    /* Start performance counter for encryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure secure context for AES-256 GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_MAXAAD_LENGTH;
    /* Copy GHASH, AAD, IV and 256-bit key for encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcmAad, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcmIv, APP_CRYPTO_AES_GCM_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAX_KEY_LENGTH);
    ctxParams.inputLen = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;
    gSa2ulCtxObj.totalLengthInBytes = TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN;

    /* Allocate context for encryption */
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform cache operations for input buffer */
    CacheP_wb(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, CacheP_TYPE_ALLD);

    /* Perform encryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmInputBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesGcmEncResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free encryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record encryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_ENCRYPT);

    /* Save authentication tag for later verification */
    (void) memcpy(gCryptoAesGcmTagBuf, gSa2ulCtxObj.computedHash, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);

    /* Start performance counter for decryption operation */
    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Configure for decryption */
    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    status = SA2UL_contextAlloc(gCryptoAesGcmContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Perform decryption */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoAesGcmEncResultBuf[0], TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, gCryptoAesGcmDecResultBuf);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Free decryption context */
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Record decryption performance */
    t2 = CycleCounterP_getCount32();
    App_fillPerformanceResults(t1, t2, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN, APP_CRYPTO_AES_GCM_256, APP_OPERATION_DECRYPT);

    /* Verify decryption results */
    status = memcmp(gCryptoAesGcmDecResultBuf, gCryptoAesGcmInputBuf, TEST_CRYPTO_AES_HW_TEST_512B_BUF_LEN);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    /* Verify authentication tag */
    status = memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcmTagBuf, APP_CRYPTO_AES_GCM_MAXAAD_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
}

static const char *bytesToString(uint64_t bytes)
{
	char *suffix[] = {"B", "KB", "MB", "GB", "TB"};
	char length = sizeof(suffix) / sizeof(suffix[0]);

	int i = 0;
	double dblBytes = bytes;

	if (bytes > 1024.0) {
        for (i = 0; ((bytes / 1024.0) > 0) && (i < (length-1)); i++, bytes /= 1024.0)
        {
			dblBytes = bytes / 1024.0;
        }
	}

	static char output[200];
	(void) sprintf(output, "%.02lf %s", dblBytes, suffix[i]);
	return output;
}

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation)
{
    uint32_t diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);
    throughputInMBps  = (double) numBytes * cpuClkMHz/(double) diffCnt;

    App_benchmark *table = &results[gCount];
    gCount++;
    table-> key = key;

    if(operation == APP_OPERATION_GHASH)
    {
        (void) strcpy(table->operation, "GHASH generation");
    }
    else if(operation == APP_OPERATION_ENCRYPT)
    {
        (void) strcpy(table->operation, "Encryption");
    }
    else
    {
        (void) strcpy(table->operation, "Decryption");
    }
    table->dataSize = numBytes;
    table->performance = (double)(8.0 * throughputInMBps);
}


void App_printPerformanceLogs()
{
    double cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    DebugP_log("BENCHMARK START - SA2UL - AES \r\n");
    DebugP_log("- Software/Application used : test_sa2ul_aes \r\n");
    DebugP_log("- Code Placement            : OCMC \r\n");
    DebugP_log("- Data Placement            : OCMC \r\n");
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