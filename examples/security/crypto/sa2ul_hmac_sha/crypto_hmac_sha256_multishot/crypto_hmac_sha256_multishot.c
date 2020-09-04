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

/* This example demonstrates the how to generate HMAC SHA-256 Multi shot hash. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* HMAC SHA-256 length */
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH                      (32U)
/*Input 2k buf length*/
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH                       (2048U)
/*Input 1k buf length*/
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_1KBUF_LENGTH                       (1024U)
/* HMAC SHA-256 key length in bytes */
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES                 (64U)
/* HMAC SHA-256 Output buffer maximun length */
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_BUFFER_MAX_SIZE_IN_BYTES    (2112U)
/* HMAC SHA-256 input key length */
#define APP_CRYPTO_HMAC_SHA256_MULTISHOT_INPUT_KEY_LENGTH                   (20U)

/*Input test buffer for hmac sha-256 computation */
static uint8_t gCryptoHmacSha256MultiShotTestInput[APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH];
/*Output test buffer for hmac sha-256 computation */
uint8_t gCryptoHmacSha256MultiShotTestOutput[APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_BUFFER_MAX_SIZE_IN_BYTES] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/*For Inner pad*/
const uint8_t  gCryptoHmacSha256MultiShotIpad[APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES];
/*For Outer pad*/
const uint8_t  gCryptoHmacSha256MultishotOpad[APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES];

/* Expected output buffer for hmac sha-256 computation */
uint8_t gCryptoHmacSha256MultiShotExpectedOutput[APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH] =
{
    0x91, 0xbe, 0xbd, 0xce, 0x88, 0x37, 0x7c, 0x4c, 0xb4, 0x1d, 0xdb, 0xab, 0xcb, 0xcd, 0x19, 0xc7, 
    0x98, 0xa5, 0xbf, 0xb3, 0x2f, 0xf6, 0x8d, 0xf3, 0xa9, 0xaa, 0xb3, 0x3b, 0x82, 0x50, 0xed, 0x97
};

/* Key for hmac sha-256 computation */
static uint8_t gCryptoHmacSha256MultiShotKey[APP_CRYPTO_HMAC_SHA256_MULTISHOT_INPUT_KEY_LENGTH] =
{
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43
};

/* For holding temporary output */
uint8_t gCryptoHmacSha256MultishotTempOut[APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH];

/* Context memory */
static Crypto_Context gCryptoHmacSha256MultiShotContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_hmac_sha256_multishot(void *args)
{
    int32_t             status;
    Crypto_Handle       hmacShaHandle;
    SA2UL_ContextParams ctxParams;
    Crypto_Params       params;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] HMAC SHA-256 Multi shot example started ...\r\n");

    /* Open HMAC SHA-256 instance */
    
    hmacShaHandle = Crypto_open(&gCryptoHmacSha256MultiShotContext);
    DebugP_assert(hmacShaHandle != NULL);

    /* Update the Ipad and Opad for HMAC SHA-256 operations */
    params.authMode                 = SA2UL_HASH_ALG_SHA2_256;
    memcpy(&params.key, gCryptoHmacSha256MultiShotKey, APP_CRYPTO_HMAC_SHA256_MULTISHOT_INPUT_KEY_LENGTH);
    params.keySizeInBytes           = sizeof(gCryptoHmacSha256MultiShotKey);
    status = Crypto_hmacSha(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA2_256;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH + APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH + APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES;

    /* Perform HMAC SHA-256 operation for ipad with input buffer */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha256MultiShotContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    memcpy((void *)gCryptoHmacSha256MultiShotIpad, params.iPad,APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES);
    memcpy((void *)gCryptoHmacSha256MultishotOpad, params.oPad,APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha256MultiShotIpad, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha256MultiShotIpad, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha256MultiShotIpad[0], APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, &gCryptoHmacSha256MultiShotTestOutput[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Setting input as all "aaaa" upto 2k */
    memset(&gCryptoHmacSha256MultiShotTestInput, 0x61, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH);
    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha256MultiShotTestInput, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha256MultiShotTestInput, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    /* Need to call this function for multiple times until input buffer reached to size of data, Here we are using 2k buffer*/
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoHmacSha256MultiShotTestInput[0], APP_CRYPTO_HMAC_SHA256_MULTISHOT_1KBUF_LENGTH, &gCryptoHmacSha256MultiShotTestOutput[0]);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoHmacSha256MultiShotTestInput[APP_CRYPTO_HMAC_SHA256_MULTISHOT_1KBUF_LENGTH], APP_CRYPTO_HMAC_SHA256_MULTISHOT_1KBUF_LENGTH, &gCryptoHmacSha256MultiShotTestOutput[APP_CRYPTO_HMAC_SHA256_MULTISHOT_1KBUF_LENGTH]);
    DebugP_assert(SystemP_SUCCESS == status);
    
    /*Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Copying intermediate output of hmac sha-256(ipad + inputbuffer) for further process*/
    memcpy(gCryptoHmacSha256MultishotTempOut, &gSa2ulCtxObj.computedHash, APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH);
    
    /* Configure secure context */
    ctxParams.opType                = SA2UL_OP_AUTH;
    ctxParams.hashAlg               = SA2UL_HASH_ALG_SHA2_256;
    ctxParams.inputLen              = APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES;
    gSa2ulCtxObj.totalLengthInBytes = APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH + APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES;
    /*  Perform HMAC SHA-256 operation for opad with output of HMAC SHA-256(ipad with input) */
    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoHmacSha256MultiShotContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb((void *)gCryptoHmacSha256MultishotOpad, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    CacheP_inv((void *)gCryptoHmacSha256MultishotOpad, APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha256MultishotOpad[0], APP_CRYPTO_HMAC_SHA256_MULTISHOT_2KBUF_KEYLEN_BYTES, &gCryptoHmacSha256MultiShotTestOutput[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(gCryptoHmacSha256MultishotTempOut, APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    CacheP_inv(gCryptoHmacSha256MultishotTempOut, APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH, CacheP_TYPE_ALLD);
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj, &gCryptoHmacSha256MultishotTempOut[0], APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH, &gCryptoHmacSha256MultiShotTestOutput[0]);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /*comparing final HMAC SHA-256 result with expected test results*/
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoHmacSha256MultiShotExpectedOutput, APP_CRYPTO_HMAC_SHA256_MULTISHOT_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] HMAC SHA-256 Multi shot example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] HMAC SHA-256 Multi shot example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Close Hmac SHA instance */
    status = Crypto_close(hmacShaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();

    return;
}