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

/* This example demonstrates the AES 256 gcm Encryption, Decryption and authentication. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input or output length*/
#define APP_CRYPTO_AES_GCM_256_INOUT_LENGTH           (51U)
/* Aes max key length*/
#define APP_CRYPTO_AES_GCM_256_MAXKEY_LENGTH          (32U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_GCM_256_MAXIV_LENGTH           (12U)
/* Aes ghash zero array length */
#define APP_CRYPTO_AES_GCM_256_ZEROARRAY_LENGTH       (16U)
/* Aes max AAD length*/
#define APP_CRYPTO_AES_GCM_256_MAXAAD_LENGTH          (16U)
/* Aes Authentication tag length*/
#define APP_CRYPTO_AES_GCM_256_AUTHTAG_LENGTH         (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_GCM_256_KEY_LENGTH_IN_BITS     (256U)

/* Input buffer for encryption or decryption */
uint8_t gCryptoAesGcm256Input[APP_CRYPTO_AES_GCM_256_INOUT_LENGTH] =
{
    0x88, 0x1d, 0xc6, 0xc7, 0xa5, 0xd4, 0x50, 0x9f,
    0x3c, 0x4b, 0xd2, 0xda, 0xab, 0x08, 0xf1, 0x65,
    0xdd, 0xc2, 0x04, 0x48, 0x9a, 0xa8, 0x13, 0x45,
    0x62, 0xa4, 0xea, 0xc3, 0xd0, 0xbc, 0xad, 0x79,
    0x65, 0x84, 0x7b, 0x10, 0x27, 0x33, 0xbb, 0x63,
    0xd1, 0xe5, 0xc5, 0x98, 0xec, 0xe0, 0xc3, 0xe5, 
    0xda, 0xdd, 0xdd
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesGcm256Key[APP_CRYPTO_AES_GCM_256_MAXKEY_LENGTH] =
{
    0x5f, 0xe0, 0x1c, 0x4b, 0xaf, 0x01, 0xcb, 0xe0,
    0x77, 0x96, 0xd5, 0xaa, 0xef, 0x6e, 0xc1, 0xf4,
    0x51, 0x93, 0xa9, 0x8a, 0x22, 0x35, 0x94, 0xae,
    0x4f, 0x0e, 0xf4, 0x95, 0x2e, 0x82, 0xe3, 0x30
};

static uint8_t gZeroArray[APP_CRYPTO_AES_GCM_256_ZEROARRAY_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Additional Authenticated Data (AAD) is an optional data that is authenticated but is not encrypted. */
static uint8_t gCryptoAesGcm256Aad[APP_CRYPTO_AES_GCM_256_MAXAAD_LENGTH] =
{
    0x90, 0x13, 0x61, 0x78, 0x17, 0xdd, 0xa9, 0x47,
    0xe1, 0x35, 0xee, 0x6d, 0xd3, 0x65, 0x33, 0x82
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesGcm256Iv[APP_CRYPTO_AES_GCM_256_MAXIV_LENGTH] = {
    0xbd, 0x58, 0x73, 0x21, 0x56, 0x6c, 0x7f, 0x1a,
    0x5d, 0xd8, 0x65, 0x2d
};

/* Test AES GCM Authentication Tag */
static const uint8_t gCryptoAesGcm256TestTag[APP_CRYPTO_AES_GCM_256_AUTHTAG_LENGTH] = {
    0xab, 0xd3, 0xd2, 0x6d, 0x65, 0xa6, 0x27, 0x5f,
    0x7a, 0x4f, 0x56, 0xb4, 0x22, 0xac, 0xab, 0x49
};

/* Encryption output buf */
uint8_t     gCryptoAesGcm256EncResultBuf[APP_CRYPTO_AES_GCM_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesGcm256DecResultBuf[APP_CRYPTO_AES_GCM_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* GHASH generated */
uint8_t     gHash[SA2UL_GHASH_LENGTH_BYTES] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
Crypto_Context gCryptoAesGcm256Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_aes_gcm_256(void *args)
{
    int32_t             status;
    Crypto_Handle       aesHandle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] AES GCM-256 example started ...\r\n");

    aesHandle = Crypto_open(&gCryptoAesGcm256Context);
    DebugP_assert(aesHandle != NULL);

    /* Configure secure context for generating GHASH */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gZeroArray);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gZeroArray);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
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
    ctxParams.aadLen       = APP_CRYPTO_AES_GCM_256_MAXAAD_LENGTH;
    /* Use the gHash generated by the ECB encryption for AES GCM Encryption */
    (void) memcpy(ctxParams.ghash, (uint8_t*)gHash, SA2UL_GHASH_LENGTH_BYTES);
    (void) memcpy(ctxParams.aad, gCryptoAesGcm256Aad, APP_CRYPTO_AES_GCM_256_MAXAAD_LENGTH);
    (void) memcpy(ctxParams.iv, gCryptoAesGcm256Iv, APP_CRYPTO_AES_GCM_256_MAXIV_LENGTH);
    (void) memcpy( &ctxParams.key[0], &gCryptoAesGcm256Key[0], APP_CRYPTO_AES_GCM_256_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gCryptoAesGcm256Input);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesGcm256Input);
    
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesGcm256Input, sizeof(gCryptoAesGcm256Input), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesGcm256Input, sizeof(gCryptoAesGcm256Input), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcm256Input[0], sizeof(gCryptoAesGcm256Input), gCryptoAesGcm256EncResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcm256TestTag, APP_CRYPTO_AES_GCM_256_AUTHTAG_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-256 example failed!!\r\n");
        status = SystemP_FAILURE;
    }
    DebugP_assert(SystemP_SUCCESS == status);

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesGcm256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesGcm256EncResultBuf[0], sizeof(gCryptoAesGcm256EncResultBuf), gCryptoAesGcm256DecResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Close AES instance */
    status = Crypto_close(aesHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gCryptoAesGcm256DecResultBuf, gCryptoAesGcm256Input, APP_CRYPTO_AES_GCM_256_INOUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-256 example failed!!\r\n");
    }
    else if(memcmp(gSa2ulCtxObj.computedHash, gCryptoAesGcm256TestTag, APP_CRYPTO_AES_GCM_256_AUTHTAG_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES GCM-256 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] AES GCM-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}