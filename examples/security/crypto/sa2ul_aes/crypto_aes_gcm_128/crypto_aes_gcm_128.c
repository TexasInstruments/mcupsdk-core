/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

/* This example demonstrates the AES 128 gcm Encryption, Decryption and authentication. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input or output length*/
#define APP_CRYPTO_AES_GCM_128_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH          (16U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_GCM_128_MAXIV_LENGTH           (12U)
/* Aes ghash zero array length */
#define APP_CRYPTO_AES_GCM_128_ZEROARRAY_LENGTH       (16U)
/* Aes max AAD length*/
#define APP_CRYPTO_AES_GCM_128_MAXAAD_LENGTH          (16U)
/* Aes Authentication tag length*/
#define APP_CRYPTO_AES_GCM_128_AUTHTAG_LENGTH         (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_GCM_128_KEY_LENGTH_IN_BITS     (128U)

/* Input buffer for encryption or decryption */
uint8_t gCryptoAesGcm128Input[APP_CRYPTO_AES_GCM_128_INOUT_LENGTH] =
{
    0xc3, 0xb3, 0xc4, 0x1f,
    0x11, 0x3a, 0x31, 0xb7,
    0x3d, 0x9a, 0x5c, 0xd4,
    0x32, 0x10, 0x30, 0x69
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesGcm128Key[APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH] =
{
    0xc9, 0x39, 0xcc, 0x13,
    0x39, 0x7c, 0x1d, 0x37,
    0xde, 0x6a, 0xe0, 0xe1,
    0xcb, 0x7c, 0x42, 0x3c
};

static uint8_t gZeroArray[APP_CRYPTO_AES_GCM_128_ZEROARRAY_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Additional Authenticated Data (AAD) is an optional data that is authenticated but is not encrypted. */
static uint8_t gCryptoAesGcm128Aad[APP_CRYPTO_AES_GCM_128_MAXAAD_LENGTH] =
{
    0x24, 0x82, 0x56, 0x02,
    0xbd, 0x12, 0xa9, 0x84,
    0xe0, 0x09, 0x2d, 0x3e,
    0x44, 0x8e, 0xda, 0x5f
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesGcm128Iv[APP_CRYPTO_AES_GCM_128_MAXIV_LENGTH] = {
    0xb3, 0xd8, 0xcc, 0x01,
    0x7c, 0xbb, 0x89, 0xb3,
    0x9e, 0x0f, 0x67, 0xe2
};

/* Test AES GCM Authentication Tag */
static const uint8_t gCryptoAesGcm128TestTag[APP_CRYPTO_AES_GCM_128_AUTHTAG_LENGTH] = {
    0x00, 0x32, 0xa1, 0xdc,
    0x85, 0xf1, 0xc9, 0x78,
    0x69, 0x25, 0xa2, 0xe7,
    0x1d, 0x82, 0x72, 0xdd
};

/* Encryption output buf */
uint8_t     gCryptoAesGcm128EncResultBuf[APP_CRYPTO_AES_GCM_128_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesGcm128DecResultBuf[APP_CRYPTO_AES_GCM_128_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* GHASH generated */
uint8_t     gHash[SA2UL_GHASH_LENGTH_BYTES] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
Crypto_Context gCryptoAesGcm128Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_aes_gcm_128(void *args)
{
    int32_t             status;
    Crypto_Handle       aesHandle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] AES GCM-128 example started ...\r\n");

    aesHandle = Crypto_open(&gCryptoAesGcm128Context);
    DebugP_assert(aesHandle != NULL);

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm128Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);
    CacheP_inv(gZeroArray, sizeof(gZeroArray), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gZeroArray[0], sizeof(gZeroArray), gHash);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure secure context for AES GCM Encryption */
    ctxParams.encMode      = SA2UL_ENC_MODE_GCM;
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_128_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcm128Aad, APP_CRYPTO_AES_GCM_128_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcm128Iv, APP_CRYPTO_AES_GCM_128_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm128Key[0], APP_CRYPTO_AES_GCM_128_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gCryptoAesGcm128Input);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesGcm128Input);
    
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm128Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcm128Input, sizeof(gCryptoAesGcm128Input), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcm128Input, sizeof(gCryptoAesGcm128Input), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcm128Input[0], sizeof(gCryptoAesGcm128Input), gCryptoAesGcm128EncResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcm128TestTag, APP_CRYPTO_AES_GCM_128_AUTHTAG_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-128 example failed!!\r\n");
        status = SystemP_FAILURE;
    }
    DebugP_assert(SystemP_SUCCESS == status);

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm128Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcm128EncResultBuf[0], sizeof(gCryptoAesGcm128EncResultBuf), gCryptoAesGcm128DecResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Close AES instance */
    status = Crypto_close(aesHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gCryptoAesGcm128DecResultBuf, gCryptoAesGcm128Input, APP_CRYPTO_AES_GCM_128_INOUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-128 example failed!!\r\n");
    }
    else if(memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcm128TestTag, APP_CRYPTO_AES_GCM_128_AUTHTAG_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-128 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] AES GCM-128 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}