/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/* This example demonstrates the implementation of DTHE HMAC-SHA512 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/crypto.h>
#include <security/crypto/dthe/dthe.h>
#include <security/crypto/dthe/dthe_sha.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input buf length*/
#define APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH             (9U)
/* HMAC SHA-512 length */
#define APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH                (64U)
/* HMAC-SHA512 input key length */
#define APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH             (64U)
/* Alignment */
#define APP_CRYPTO_HMAC_SHA512_BUF_ALIGNMENT                (128U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                              (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                          (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                          (0xCE005000U)

/* Input test buffer for hmac sha-512 computation */
static uint8_t gCryptoHmacSha512TestInputBuf[APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH] = {"abcdefpra"};

/* Expected output buffer for hmac sha-512 computation */
uint8_t gCryptoHmacSha512ExpectedOutput[APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH] =
{
    0xedU, 0x5aU, 0x6fU, 0xe2U, 0x41U, 0x32U, 0xb9U, 0xfaU, 
    0xa7U, 0x1fU, 0x3dU, 0x3eU, 0xf0U, 0xcaU, 0x0aU, 0x42U, 
    0xefU, 0x82U, 0xf8U, 0xfeU, 0x0dU, 0xf5U, 0x9fU, 0x35U, 
    0x1fU, 0x26U, 0xdbU, 0x10U, 0x22U, 0xc8U, 0x94U, 0x7aU, 
    0x1dU, 0xeeU, 0x7aU, 0x3aU, 0xa4U, 0x91U, 0x71U, 0x8bU, 
    0x85U, 0xbcU, 0x4dU, 0x8aU, 0x7aU, 0xd1U, 0xadU, 0xe5U, 
    0x0fU, 0xc1U, 0x87U, 0x94U, 0x6fU, 0x56U, 0x38U, 0x41U, 
    0x11U, 0x99U, 0xcfU, 0xe8U, 0x67U, 0x44U, 0x27U, 0xc7U
};

/* Key buffer for hmac sha-512 computation */
static uint8_t gCryptoHmacSha512Key[APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH] =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU,
    0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU,
    0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U, 0x27U, 0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2DU, 0x2EU, 0x2FU,
    0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U, 0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU
};

void crypto_hmac_sha512_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_SHA_Return_t   status;
    DTHE_Handle         shaHandle;
    DTHE_SHA_Params     shaParams;

    DebugP_log("[CRYPTO] DTHE HMAC SHA-512 example started ...\r\n");

    /* opens DTHe driver */
    shaHandle = DTHE_open(0);
    DebugP_assert(shaHandle != NULL);

    /* Opening sha driver */
    status = DTHE_SHA_open(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Initialize the SHA Parameters */
    shaParams.algoType          = DTHE_SHA_ALGO_SHA512;
    shaParams.ptrDataBuffer     = (uint32_t*)&gCryptoHmacSha512TestInputBuf[0];
    shaParams.dataLenBytes      = APP_CRYPTO_HMAC_SHA512_INPUT_BUF_LENGTH;
    shaParams.ptrKey            = (uint32_t*)&gCryptoHmacSha512Key[0];
    shaParams.keySize           = APP_CRYPTO_HMAC_SHA512_INPUT_KEY_LENGTH;

    /* Performing DTHE HMAC SHA operation */
    status = DTHE_HMACSHA_compute(shaHandle, &shaParams);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Closing sha driver */
    status = DTHE_SHA_close(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(shaHandle))
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_SHA_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Comparing final HMAC SHA-512 result with expected test results*/
    if(memcmp(shaParams.digest, gCryptoHmacSha512ExpectedOutput, APP_CRYPTO_HMAC_SHA512_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] DTHE HMAC SHA-512 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] DTHE HMAC SHA-512 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

/* Public context crypto dthe, aes and sha accelerators base address */
DTHE_Attrs gDTHE_Attrs[1] =
{
    {
        /* crypto accelerator base address */
        .caBaseAddr         = CSL_DTHE_PUBLIC_U_BASE,
        /* AES base address */
        .aesBaseAddr        = CSL_DTHE_PUBLIC_AES_U_BASE,
        /* SHA base address */
        .shaBaseAddr        = CSL_DTHE_PUBLIC_SHA_U_BASE,
        /* For checking dthe driver open or close */
        .isOpen             = FALSE,
    },
};

DTHE_Config gDtheConfig[1]=
{
    {
        &gDTHE_Attrs[0],
    },
};

uint32_t gDtheConfigNum = 1;