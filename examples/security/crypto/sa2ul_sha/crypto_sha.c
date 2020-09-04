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

/* This example demonstrates the implementation of SHA */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* SHA512 length */
#define APP_SHA512_LENGTH               (64U)
/* SHA256 length */
#define APP_SHA256_LENGTH               (32U)
/* Alignment */
#define APP_SHA_ALIGNMENT_FOR_BUF       (128U)
/*Input and output buf length*/
#define APP_SHA_IN_OUT_BUF_LENGTH       (9U)

/*Inout test buffer for sha computation */
static uint8_t gCryptoShaTestInputBuf[APP_SHA_IN_OUT_BUF_LENGTH] = {"abcdefpra"};
/*Output test buffer for sha computation */
uint8_t gCryptoShaTestOutputBuf[APP_SHA_IN_OUT_BUF_LENGTH] __attribute__ ((aligned (APP_SHA_ALIGNMENT_FOR_BUF)));

/* SHA-512 test vectors,this is expected hash for the test buffer */
static uint8_t gCryptoSha512TestSum[APP_SHA512_LENGTH] =
{
    0x1d, 0xa6, 0x8d, 0x60, 0x62, 0x9b, 0x61, 0xf4,
    0xab, 0x16, 0x67, 0x77, 0x2a, 0x21, 0xae, 0x85,
    0xbf, 0x5c, 0x20, 0xdf, 0x5c, 0x38, 0xcc, 0xa5,
    0x29, 0xc6, 0xce, 0x09, 0x22, 0xbe, 0x15, 0x7f,
    0x04, 0x9c, 0x22, 0x0a, 0xab, 0x85, 0xb4, 0x3c,
    0x49, 0x66, 0x12, 0xfa, 0x12, 0xd5, 0x41, 0xac,
    0x78, 0x50, 0x9a, 0x5f, 0x03, 0x4c, 0xc9, 0xcb,
    0x64, 0x39, 0x0a, 0x74, 0x2a, 0xb6, 0xab, 0x43
};

/* SHA-256 test vectors,this is expected hash for the test buffer */
static uint8_t gCryptoSha256TestSum[APP_SHA256_LENGTH] =
{
    0xC2, 0x77, 0xB1, 0x18, 0x8B, 0x34, 0xF5, 0x95,
    0x3A, 0x50, 0xA5, 0x28, 0xE4, 0xD8, 0x68, 0x01,
    0x14, 0xFB, 0xED, 0x82, 0x56, 0x85, 0x6D, 0xA2,
    0xFD, 0x82, 0x35, 0xD5, 0x00, 0x71, 0x96, 0x6F
};

/* Context memory */
static Crypto_Context gCryptoShaContext __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

void crypto_sha(void *args)
{
    int32_t             status;
    Crypto_Handle       shaHandle;
    SA2UL_ContextParams ctxParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[CRYPTO] SHA example started ...\r\n");

    shaHandle = Crypto_open(&gCryptoShaContext);
    DebugP_assert(shaHandle != NULL);

    /* Configure secure context */
    ctxParams.opType    = SA2UL_OP_AUTH;
    ctxParams.hashAlg   = SA2UL_HASH_ALG_SHA2_512;
    ctxParams.inputLen  = sizeof(gCryptoShaTestInputBuf);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoShaTestInputBuf);

    status = SA2UL_contextAlloc(gCryptoShaContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform SHA operation */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoShaTestInputBuf[0], sizeof(gCryptoShaTestInputBuf), gCryptoShaTestOutputBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /*comparing result with expected test results*/
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoSha512TestSum, APP_SHA512_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] SHA-512 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] SHA-512 example completed!!\r\n");
    }

    /* Configure secure context */
    ctxParams.opType    = SA2UL_OP_AUTH;
    ctxParams.hashAlg   = SA2UL_HASH_ALG_SHA2_256;
    ctxParams.inputLen  = sizeof(gCryptoShaTestInputBuf);
    gSa2ulCtxObj.totalLengthInBytes = sizeof(gCryptoShaTestInputBuf);

    status = SA2UL_contextAlloc(gCryptoShaContext.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform SHA operation */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&gCryptoShaTestInputBuf[0], sizeof(gCryptoShaTestInputBuf), gCryptoShaTestOutputBuf);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    /*comparing result with expected test results*/
    if(memcmp(gSa2ulCtxObj.computedHash, gCryptoSha256TestSum, APP_SHA256_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] SHA-256 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] SHA-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Close SHA instance */
    status = Crypto_close(shaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();

    return;
}