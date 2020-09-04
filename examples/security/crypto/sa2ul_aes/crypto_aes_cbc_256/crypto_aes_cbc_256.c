/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* This example demonstrates the AES 256 cbc Encryption and Decryptions. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input or output length*/
#define APP_CRYPTO_AES_CBC_256_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH         (32U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_CBC_256_MAXIV_LENGTH           (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_CBC_256_KEY_LENGTH_IN_BITS     (256U)

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCbc256InputBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] =
{
    0x81, 0xEA, 0x5B, 0xA4, 0x69, 0x45, 0xC1, 0x70,
    0x5F, 0x6F, 0x89, 0x77, 0x88, 0x68, 0xCC, 0x67
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCbc256Key[APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCbc256Iv[APP_CRYPTO_AES_CBC_256_MAXIV_LENGTH] =
{
    0x7C, 0xE2, 0xAB, 0xAF, 0x8B, 0xEF, 0x23, 0xC4,
    0x81, 0x6D, 0xC8, 0xCE, 0x84, 0x20, 0x48, 0xA7
};
/* Encryption output buf */
uint8_t     gCryptoAesCbc256EncResultBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesCbc256DecResultBuf[APP_CRYPTO_AES_CBC_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
static Crypto_Context gCryptoAesCbcContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_aes_cbc_256(void *args)
{
    int32_t             status;
    Crypto_Handle       aesHandle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] AES CBC-256 example started ...\r\n");

    aesHandle = Crypto_open(&gCryptoAesCbcContext);
    DebugP_assert(aesHandle != NULL);

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc256Key[0], APP_CRYPTO_AES_CBC_256_MAX_KEY_LENGTH);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbc256Iv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gCryptoAesCbc256InputBuf);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesCbc256InputBuf);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbc256InputBuf, sizeof(gCryptoAesCbc256InputBuf), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbc256InputBuf, sizeof(gCryptoAesCbc256InputBuf), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbc256InputBuf[0], sizeof(gCryptoAesCbc256InputBuf), gCryptoAesCbc256EncResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbcContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbc256EncResultBuf[0], sizeof(gCryptoAesCbc256EncResultBuf), gCryptoAesCbc256DecResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Close AES instance */
    status = Crypto_close(aesHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gCryptoAesCbc256DecResultBuf, gCryptoAesCbc256InputBuf, APP_CRYPTO_AES_CBC_256_INOUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CBC-256 example failed!!\r\n");
    }

    else
    {
        DebugP_log("[CRYPTO] AES CBC-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}