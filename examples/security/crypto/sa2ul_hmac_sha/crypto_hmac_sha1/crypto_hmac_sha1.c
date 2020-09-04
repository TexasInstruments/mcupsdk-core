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

/* This example demonstrates the implementation of HMAC SHA1 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* HMAC SHA1 length */
#define APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH                 (20U)
/* Input buf length*/
#define APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH              (9U)
/* HMAC SHA1 key length */
#define APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES                  (64U)
/* HMAC SHA1 output buffer Maximum length */
#define APP_CRYPTO_HMAC_SHA1_OUTPUT_BUFFER_MAX_LENGTH      (84U)
/* HMAC SHA1 input Key length */
#define APP_CRYPTO_HMAC_SHA1_INPUT_KEY_LENGTH              (20U)

/*Input test buffer for hmac sha1  computation */
static uint8_t gCryptoHmacSha1TestInputBuf[APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH] = {"abcdefpra"};
/*Output test buffer for hmac sha1  computation */
uint8_t gCryptoHmacSha1TestOutputBuf[APP_CRYPTO_HMAC_SHA1_OUTPUT_BUFFER_MAX_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/*For Inner pad*/
const uint8_t  gCryptoHmacSha1Ipad[APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES];
/*For Outer pad*/
const uint8_t  gCryptoHmacSha1Opad[APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES];

/* Expected output buffer for hmac sha1 computation */
uint8_t gCryptoHmacSha1ExpectedOutput[APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH] =
{
    0x7c, 0xb5, 0x18, 0x18, 0x17, 0xab, 0x4e, 0x6d, 0x46, 0x89,
    0x0f, 0x69, 0xab, 0xbd, 0x99, 0xb8, 0x53, 0xfc, 0x3d, 0x97
};

/* Key buffer for hmac sha1 computation */
static uint8_t gCryptoHmacSha1Key[APP_CRYPTO_HMAC_SHA1_INPUT_KEY_LENGTH] =
{
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43
};

/* For holding temporary output */
uint8_t gCryptoHmacSha1TempOut[APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH];

/* Context memory */
static Crypto_Context gCryptoHmacSha1Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_hmac_sha1(void *args)
{
    int32_t             status;
    Crypto_Handle       shaHandle;
    SA2UL_ContextParams ctxParams;
    Crypto_Params       params;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] HMAC SHA-1 example started ...\r\n");

    /* Open HMAC SHA1 instance */
    shaHandle = Crypto_open(&gCryptoHmacSha1Context);
    DebugP_assert(shaHandle != NULL);

    /* Update the Ipad and Opad for HMAC SHA1  operations */
    params.authMode                 = SA2UL_HASH_ALG_SHA1;
    memcpy(&params.key, gCryptoHmacSha1Key, APP_CRYPTO_HMAC_SHA1_INPUT_KEY_LENGTH);
    params.keySizeInBytes           = sizeof(gCryptoHmacSha1Key);
    status = Crypto_hmacSha(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA1;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH + APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH + APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES;

    /* Perform HMAC SHA1  operation for ipad with input buffer */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha1Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    memcpy((void *)gCryptoHmacSha1Ipad, params.iPad,APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES);
    memcpy((void *)gCryptoHmacSha1Opad, params.oPad,APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha1Ipad, APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha1Ipad, APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha1Ipad[0], APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, gCryptoHmacSha1TestOutputBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha1TestInputBuf, APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha1TestInputBuf, APP_CRYPTO_HMAC_SHA1_INPUT_BUF_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoHmacSha1TestInputBuf[0], sizeof(gCryptoHmacSha1TestInputBuf), &gCryptoHmacSha1TestOutputBuf[APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Copying intermediate output of hmac sha1(ipad + inputbuffer) for further process*/
    memcpy(gCryptoHmacSha1TempOut, &gSa2ulCtxObj.computedHash, APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH);

    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA1;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES;
    /* Perform HMAC SHA1  operation for opad with output of HMAC SHA1(ipad with input) */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha1Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha1Opad, APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha1Opad, APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha1Opad[0], APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES, &gCryptoHmacSha1TestOutputBuf[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha1TempOut, APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha1TempOut, APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha1TempOut[0], APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH, &gCryptoHmacSha1TestOutputBuf[APP_CRYPTO_HMAC_SHA1_KEYLEN_BYTES]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Comparing final HMAC SHA1  result with expected test results*/
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoHmacSha1ExpectedOutput, APP_CRYPTO_HMAC_SHA1_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] HMAC SHA-1 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] HMAC SHA-1 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Close HMAC SHA instance */
    status = Crypto_close(shaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();

    return;
}