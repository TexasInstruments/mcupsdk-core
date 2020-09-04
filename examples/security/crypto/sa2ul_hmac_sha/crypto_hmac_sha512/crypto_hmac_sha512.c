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

/* This example demonstrates the implementation of HMAC-SHA512 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* HMAC SHA-512 length */
#define APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH                  (64U)
/*Input buf length*/
#define APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH                 (9U)
/* HMAC SHA-512 key length */
#define APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES                  (128U)
/* HMAC SHA-512 output buffer maximum length */
#define APP_CRYPTO_HMAC_SHA512_OUTPUT_BUFFER_MAX_LENGTH         (192U)
/* HMAC-SHA512 input key length */
#define APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH                 (20U)

/*Input test buffer for hmac sha-512 computation */
static uint8_t gCryptoHmacSha512TestInputBuf[APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH] = {"abcdefpra"};
/*Output test buffer for hmac sha computation */
uint8_t gCryptoHmacSha512TestOutputBuf[APP_CRYPTO_HMAC_SHA512_OUTPUT_BUFFER_MAX_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/*For Inner pad*/
const uint8_t  gCryptoHmacSha512Ipad[APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES];
/*For outer pad*/
const uint8_t  gCryptoHmacSha512Opad[APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES];

/* Expected output buffer for hmac sha-512 computation */
uint8_t gCryptoHmacSha512ExpectedOutput[APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH] =
{
    0x5d, 0x9c, 0xf2, 0x02, 0xdf, 0xf4, 0x35, 0xce, 0x3f, 0xab,
    0x42, 0xcf, 0x35, 0xde, 0x4e, 0xc4, 0x32, 0xf2, 0x90, 0x87,
    0xf2, 0xef, 0xb5, 0x28, 0x89, 0xb6, 0xb2, 0xba, 0xe9, 0xb4,
    0x01, 0xac, 0xd3, 0x94, 0xd8, 0xd6, 0x3c, 0x3f, 0x3e, 0xbd,
    0x8b, 0xe2, 0xdb, 0x8f, 0x85, 0x54, 0xd2, 0xb2, 0x45, 0xbf,
    0x56, 0x95, 0x5e, 0x8d, 0xa3, 0x9e, 0xef, 0x01, 0xd2, 0x93,
    0x9c, 0xbb, 0x5e, 0x6f
};

/* Key buffer for hmac sha-512 computation */
static uint8_t gCryptoHmacSha512Key[APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH] =
{
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43
};

/* For holding temporary output */
uint8_t gCryptoHmacSha512TempOut[APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH];

/* Context memory */
static Crypto_Context gCryptoHmacSha512Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_hmac_sha512(void *args)
{
    int32_t             status;
    Crypto_Handle       shaHandle;
    SA2UL_ContextParams ctxParams;
    Crypto_Params       params;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] HMAC SHA-512 example started ...\r\n");

    /* Open HMAC SHA-512 instance */
    shaHandle = Crypto_open(&gCryptoHmacSha512Context);
    DebugP_assert(shaHandle != NULL);

    /* Update the Ipad and Opad for HMAC SHA operations */
    params.authMode                 = SA2UL_HASH_ALG_SHA2_512;
    memcpy(&params.key, gCryptoHmacSha512Key, APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH);
    params.keySizeInBytes           = sizeof(gCryptoHmacSha512Key);
    status = Crypto_hmacSha(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA2_512;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH + APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH + APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES;

    /*  Perform HMAC SHA-512 operation for ipad with input buffer */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha512Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    memcpy((void *)gCryptoHmacSha512Ipad, params.iPad,APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES);
    memcpy((void *)gCryptoHmacSha512Opad, params.oPad,APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha512Ipad, APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha512Ipad, APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha512Ipad[0], APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, gCryptoHmacSha512TestOutputBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha512TestInputBuf, APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha512TestInputBuf, APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoHmacSha512TestInputBuf[0], sizeof(gCryptoHmacSha512TestInputBuf), &gCryptoHmacSha512TestOutputBuf[APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES]);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Copying intermediate output of hmac sha-512(ipad + inputbuffer) for further process*/
    memcpy(gCryptoHmacSha512TempOut, &gSa2ulCtxObj.computedHash, APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH);

    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA2_512;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES;
    /*  Perform HMAC SHA-512 operation for opad with output of HMAC SHA-512(ipad with input) */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha512Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha512Opad, APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha512Opad, APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha512Opad[0], APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES, &gCryptoHmacSha512TestOutputBuf[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha512TempOut, APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha512TempOut, APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha512TempOut[0], APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH, &gCryptoHmacSha512TestOutputBuf[APP_CRYPTO_HMAC_SHA512_KEYLEN_IN_BYTES]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /*comparing final HMAC SHA-512 result with expected test results*/
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoHmacSha512ExpectedOutput, APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] HMAC SHA-512 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] HMAC SHA-512 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Close HMAC SHA instance */
    status = Crypto_close(shaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();

    return;
}