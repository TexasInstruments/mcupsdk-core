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

/* This example demonstrates the AES 256 ecb Encryption and Decryptions. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input or output length*/
#define APP_CRYPTO_AES_ECB_256_INOUT_LENGTH           (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_256_MAXKEY_LENGTH          (32U)
/* Aes key length in bites*/
#define APP_CRYPTO_AES_ECB_256_KEY_LENGTH_IN_BITS     (256U)

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesEcb256Input[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH] =
{
    0x57, 0xB1, 0x8D, 0xFE, 0xAD, 0x12, 0x97, 0x95,
    0xC3, 0xBB, 0x8D, 0x6C, 0x06, 0x76, 0x47, 0x29
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesEcb256Key[APP_CRYPTO_AES_ECB_256_MAXKEY_LENGTH] =
{
    0x0B, 0xE3, 0xFC, 0x6C, 0x21, 0x76, 0x4F, 0x1C,
    0x15, 0x78, 0x2D, 0x52, 0xD9, 0x29, 0x19, 0xA0,
    0x9B, 0x10, 0x1F, 0xAE, 0x15, 0xE1, 0xC9, 0xE4,
    0xC8, 0x0B, 0x13, 0x13, 0xC2, 0x91, 0x96, 0x60
};

/* Encryption output buf */
uint8_t     gCryptoAesEcb256EncResultBuf[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Decryption output buf */
uint8_t     gCryptoAesEcb256DecResultBuf[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context memory */
static Crypto_Context gCryptoAesEcb256Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_aes_ecb_256(void *args)
{
    int32_t             status;
    Crypto_Handle       aesHandle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] AES ECB-256 example started ...\r\n");

    aesHandle = Crypto_open(&gCryptoAesEcb256Context);
    DebugP_assert(aesHandle != NULL);

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &gCryptoAesEcb256Key[0], APP_CRYPTO_AES_ECB_256_MAXKEY_LENGTH);
    ctxParams.inputLen = sizeof(gCryptoAesEcb256Input);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoAesEcb256Input);

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesEcb256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoAesEcb256Input, sizeof(gCryptoAesEcb256Input), CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoAesEcb256Input, sizeof(gCryptoAesEcb256Input), CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesEcb256Input[0], sizeof(gCryptoAesEcb256Input), gCryptoAesEcb256EncResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    ctxParams.encDirection = SA2UL_ENC_DIR_DECRYPT;
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesEcb256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Decryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoAesEcb256EncResultBuf[0], sizeof(gCryptoAesEcb256EncResultBuf), gCryptoAesEcb256DecResultBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Close AES instance */
    status = Crypto_close(aesHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing result with expected test results */
    if(memcmp(gCryptoAesEcb256DecResultBuf, gCryptoAesEcb256Input, APP_CRYPTO_AES_ECB_256_INOUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES ECB-256 example failed!!\r\n");
    }

    else
    {
        DebugP_log("[CRYPTO] AES ECB-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}