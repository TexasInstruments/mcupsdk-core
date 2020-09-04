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

/* This example demonstrates the AES 128 cbc Encryption and Decryptions. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input or output length*/
#define APP_CRYPTO_AES_CBC_128_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_CBC_128_MAXKEY_LENGTH          (16U)
/* Aes max IV length*/
#define APP_CRYPTO_AES_CBC_128_MAXIV_LENGTH           (16U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_CBC_128_KEY_LENGTH_IN_BITS     (128U)

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCbc128Input[APP_CRYPTO_AES_CBC_128_INOUT_LENGTH] =
{
    0x98, 0x3B, 0xF6, 0xF5, 0xA6, 0xDF, 0xBC, 0xDA,
    0xA1, 0x93, 0x70, 0x66, 0x6E, 0x83, 0xA9, 0x9A
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCbc128Key[APP_CRYPTO_AES_CBC_128_MAXKEY_LENGTH] =
{
    0x93, 0x28, 0x67, 0x64, 0xA8, 0x51, 0x46, 0x73,
    0x0E, 0x64, 0x18, 0x88, 0xDB, 0x34, 0xEB, 0x47
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCbc128Iv[APP_CRYPTO_AES_CBC_128_MAXIV_LENGTH] =
{
    0x19, 0x2D, 0x9B, 0x3A, 0xA1, 0x0B, 0xB2, 0xF7,
    0x84, 0x6C, 0xCB, 0xA0, 0x08, 0x5C, 0x65, 0x7A
};

/* Encryption output buf */
uint8_t     gCryptoAesCbc128EncResultBuf[APP_CRYPTO_AES_CBC_128_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesCbc128DecResultBuf[APP_CRYPTO_AES_CBC_128_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
static Crypto_Context gCryptoAesCbc128Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_aes_cbc_128(void *args)
{
    int32_t             status;
    Crypto_Handle       handle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] AES CBC-128 example started ...\r\n");

    handle = Crypto_open(&gCryptoAesCbc128Context);
    DebugP_assert(handle != NULL);

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_CBC;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_128;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesCbc128Key[0], APP_CRYPTO_AES_CBC_128_MAXKEY_LENGTH);
    memcpy( &ctxParams.iv[0], &gCryptoAesCbc128Iv[0], SA2UL_MAX_IV_SIZE_BYTES);
    ctxParams.inputLen = sizeof(gCryptoAesCbc128Input);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesCbc128Input);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbc128Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesCbc128Input, sizeof(gCryptoAesCbc128Input), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesCbc128Input, sizeof(gCryptoAesCbc128Input), CacheP_TYPE_ALLD);
    
    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbc128Input[0], sizeof(gCryptoAesCbc128Input), gCryptoAesCbc128EncResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesCbc128Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesCbc128EncResultBuf[0], sizeof(gCryptoAesCbc128EncResultBuf), gCryptoAesCbc128DecResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Close AES instance */
    status = Crypto_close(handle);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gCryptoAesCbc128DecResultBuf, gCryptoAesCbc128Input, APP_CRYPTO_AES_CBC_128_INOUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CBC-128 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] AES CBC-128 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}