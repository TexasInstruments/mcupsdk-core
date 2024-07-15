/*
 *  Copyright (C) 2022-24 Texas Instruments Incorporated
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

/* This example demonstrates the DTHE AES 256 ecb Encryption and Decryptions. */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/security_common/drivers/crypto/crypto.h>
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_aes.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Input or output length*/
#define APP_CRYPTO_AES_ECB_256_INOUT_LENGTH             (16U)
/* AES ECB KEY length in bytes */
#define APP_CRYPTO_AES_ECB_256_KEY_LENGTH_IN_BYTES      (32U)
/* AES ECB KEY Catche alignment size */
#define APP_CRYPTO_AES_ECB_256_CATCHE_ALIGNMENT         (32U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                          (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                      (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                      (0xCE005000U)

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                       (1U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesEcb256PlainText[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_ECB_256_CATCHE_ALIGNMENT))) =
{
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesEcb256Key[APP_CRYPTO_AES_ECB_256_KEY_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_ECB_256_CATCHE_ALIGNMENT))) =
{
    0xc4U, 0x7bU, 0x02U, 0x94U, 0xdbU, 0xbbU, 0xeeU, 0x0fU,
    0xecU, 0x47U, 0x57U, 0xf2U, 0x2fU, 0xfeU, 0xeeU, 0x35U,
    0x87U, 0xcaU, 0x47U, 0x30U, 0xc3U, 0xd3U, 0x3bU, 0x69U,
    0x1dU, 0xf3U, 0x8bU, 0xabU, 0x07U, 0x6bU, 0xc5U, 0x58U
};

/* Encrypted buffer of gCryptoAesEcb256Key */
static uint8_t gCryptoAesEcb256CipherText[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_ECB_256_CATCHE_ALIGNMENT))) =
{
    0x46U, 0xf2U, 0xfbU, 0x34U, 0x2dU, 0x6fU, 0x0aU, 0xb4U,
    0x77U, 0x47U, 0x6fU, 0xc5U, 0x01U, 0x24U, 0x2cU, 0x5fU
};

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

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
        DMA_DISABLE,
    },
};
uint32_t gDtheConfigNum = 1;

DMA_Config gDmaConfig[1]=
{
    {
        &gEdmaHandle[0],
        &gEdmaFxns,
    },
};
uint32_t gDmaConfigNum = 1;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void crypto_aes_ecb_256_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_AES_Return_t   status;
    DTHE_Handle         aesHandle;
    DTHE_AES_Params     aesParams;
    uint32_t            aesResult[APP_CRYPTO_AES_ECB_256_INOUT_LENGTH/4U];

    DebugP_log("[CRYPTO] DTHE AES ECB-256 example started ...\r\n");

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResult[0], 0xFF, sizeof(aesResult));

    /* Initialize the decryption parameters: */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesEcb256CipherText[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_ECB_256_INOUT_LENGTH;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult[0];
    aesParams.useKEKMode        = FALSE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesEcb256PlainText[0], (void *)&aesResult[0], APP_CRYPTO_AES_ECB_256_INOUT_LENGTH) == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        /* Initialize the AES Parameters */
        (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the results: We set the result to a non-zero value */
        (void)memset ((void *)&aesResult[0], 0xFF, sizeof(aesResult));

        /* Initialize the encryption parameters */
        aesParams.algoType          = DTHE_AES_ECB_MODE;
        aesParams.opType            = DTHE_AES_ENCRYPT;
        aesParams.ptrKey            = (uint32_t*)&gCryptoAesEcb256Key[0];
        aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
        aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesEcb256PlainText[0];
        aesParams.dataLenBytes      = APP_CRYPTO_AES_ECB_256_INOUT_LENGTH;
        aesParams.ptrEncryptedData  = (uint32_t*)&aesResult[0];
        aesParams.useKEKMode        = FALSE;

        /* Encryption */
        status = DTHE_AES_execute(aesHandle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(aesHandle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesEcb256CipherText[0], (void *)&aesResult[0], APP_CRYPTO_AES_ECB_256_INOUT_LENGTH) == 0)
    {
        DebugP_log("[CRYPTO] DTHE AES ECB-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[CRYPTO] DTHE AES ECB-256 example failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}
