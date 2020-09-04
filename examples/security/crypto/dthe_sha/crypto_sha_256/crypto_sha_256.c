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

/* This example demonstrates the implementation of DTHE SHA */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/crypto.h>
#include <security/crypto/dthe/dthe.h>
#include <security/crypto/dthe/dthe_sha.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* SHA256 length */
#define APP_CRYPTO_SHA256_LENGTH                (32U)
/* Alignment */
#define APP_CRYPTO_SHA_BUF_ALIGNMENT            (128U)
/*Input and output buf length*/
#define APP_CRYPTO_SHA_INPUT_BUF_LENGTH         (9U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                  (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE              (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE              (0xCE005000U)

/*Inout test buffer for sha computation */
static uint8_t gCryptoShaTestInputBuf[APP_CRYPTO_SHA_INPUT_BUF_LENGTH] = {"abcdefpra"};

/* SHA-256 test vectors,this is expected hash for the test buffer */
static uint8_t gCryptoSha256TestSum[APP_CRYPTO_SHA256_LENGTH] =
{
    0xC2, 0x77, 0xB1, 0x18, 0x8B, 0x34, 0xF5, 0x95,
    0x3A, 0x50, 0xA5, 0x28, 0xE4, 0xD8, 0x68, 0x01,
    0x14, 0xFB, 0xED, 0x82, 0x56, 0x85, 0x6D, 0xA2,
    0xFD, 0x82, 0x35, 0xD5, 0x00, 0x71, 0x96, 0x6F
};

void crypto_sha_256_main(void *args)
{
    Drivers_open();
    Board_driversOpen();
    
    DTHE_SHA_Return_t   status;
    DTHE_Handle         shaHandle;
    DTHE_SHA_Params     shaParams;

    DebugP_log("[CRYPTO] SHA example started ...\r\n");

    /* Opening crypto driver */
    shaHandle = DTHE_open(0);
    DebugP_assert(shaHandle != NULL);

    /* Opening sha driver */
    status = DTHE_SHA_open(shaHandle);
    DebugP_assert(DTHE_SHA_RETURN_SUCCESS == status);

    /* Initialize the AES Parameters */
    shaParams.algoType          = DTHE_SHA_ALGO_SHA256;
    shaParams.ptrDataBuffer     = (uint32_t*)&gCryptoShaTestInputBuf[0];
    shaParams.dataLenBytes      = APP_CRYPTO_SHA_INPUT_BUF_LENGTH;

    /* Performing DTHE SHA operation */
    status = DTHE_SHA_compute(shaHandle, &shaParams, TRUE);
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

    /*comparing result with expected test results*/
    if(memcmp(shaParams.digest, gCryptoSha256TestSum, APP_CRYPTO_SHA256_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] SHA-256 example failed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] SHA-256 example completed!!\r\n");
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