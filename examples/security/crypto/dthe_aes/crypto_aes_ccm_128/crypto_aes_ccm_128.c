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

/* This example demonstrates the DTHE AES 128 cbc Encryption and Decryptions. */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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
#define APP_CRYPTO_AES_CCM_128_INOUT_LENGTH             (4U)
/* AES CCM IV length in bytes */
#define APP_CRYPTO_AES_CCM_128_NONCE_LENGTH_IN_WORD     (8U)
/* AES CCM KEY length in bytes */
#define APP_CRYPTO_AES_CCM_128_KEY_LENGTH_IN_WORDS      (16U)
/* AES CCM KEY Catche alignment size */
#define APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT         (32U)
/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                       (1U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                          (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                      (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                      (0xCE005000U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];
/* Input buffer for encryption or decryption */
static uint32_t gCryptoAesCcm128PlainText[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0x23222120, 0x27262524, 0x2b2a2928, 0x2f2e2d2c
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint32_t gCryptoAesCcm128Key[APP_CRYPTO_AES_CCM_128_KEY_LENGTH_IN_WORDS] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0x43424140,0x47464544,0x4b4a4948,0x4f4e4d4c
};

/* Encrypted buffer of gCryptoAesCcm128PlainText */
static uint32_t gCryptoAesCcm128CipherText[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0xe0f0a1d2, 0x625fea51, 0x92771a08, 0x3d593d07
};

/* Nonce is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
/* First byte should be Nonce CCML value, app_aes_ccm_getCCM_L() for getting ccmL value*/
/* Example Nonce lengght is 8 bytes, ccmL = app_aes_ccm_getCCM_L(8), ccmL = 0x6 put a 0th index of nonce*/
static uint32_t gCryptoAesCcm128Nonce[APP_CRYPTO_AES_CCM_128_NONCE_LENGTH_IN_WORD] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0x12111006, 0x16151413, 0x00000017, 0x00000000
};

/* AAD is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint32_t gCryptoAesCcm128AAD[APP_CRYPTO_AES_CCM_128_NONCE_LENGTH_IN_WORD] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c
};
/* Expected tag */
static uint32_t gAesCmac128ExpectedTag[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_128_CATCHE_ALIGNMENT))) =
{
    0xbf4fc61f, 0x0000cdac, 0x00000000, 0x00000000
};

/* Tag holder */
static uint32_t gCryptoAesCCM128Tag[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH];
/* Decrypted Data holder */
static uint32_t gCryptoAesCCM128DecData[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH];


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
/*                          Local Function Declarations                       */
/* ========================================================================== */
uint32_t app_aes_ccm_getCCM_L(uint16_t noOnceLenght);
uint32_t app_aes_ccm_getCCM_M(uint16_t tagLenght);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void crypto_aes_ccm_128_main(void *args)
{
    DTHE_AES_Return_t       status;
    DTHE_Handle             handle;
    DTHE_AES_Params         aesParams;
    uint32_t                aesResult[APP_CRYPTO_AES_CCM_128_INOUT_LENGTH];

    /* opens DTHe driver */
    handle = DTHE_open(0);
    DebugP_assert(handle != NULL);

    DebugP_log("[CRYPTO] DTHE AES CCM-128 example started ...\r\n");

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResult[0], 0xFF, sizeof(aesResult));

    /* opens aes driver */
    status = DTHE_AES_open(handle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CCM_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCcm128Key[0];
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCcm128Nonce[0];
    aesParams.dataLenBytes      = 16U;
    aesParams.useKEKMode        = FALSE;
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.aadLength         = 16;
    aesParams.ptrAAD            = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCcm128AAD[0]);
    aesParams.ptrTag            = (uint32_t*)NULL;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;
    aesParams.streamState       = DTHE_AES_STREAM_INIT;
    aesParams.streamSize        = 0U;
    aesParams.ccmL              = app_aes_ccm_getCCM_L((uint16_t)8);/* Pass nonce lenght in bytes*/
    aesParams.ccmM              = app_aes_ccm_getCCM_M((uint16_t)6);/* Pass tag lenght in bytes*/

    /* Encryption */
    status = DTHE_AES_execute(handle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.ptrPlainTextData  = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCcm128PlainText[0]);
    aesParams.ptrEncryptedData  = (uint32_t*)SOC_virtToPhy((void *)&aesResult[0]);
    aesParams.aadLength         = 0;
    aesParams.ptrTag            = (uint32_t*)&gCryptoAesCCM128Tag[0];
    aesParams.streamState       = DTHE_AES_STREAM_FINISH;
    aesParams.streamSize        = 16U;
    aesParams.dataLenBytes      = 16U;

    /* Sending plaintext */
    status = DTHE_AES_execute(handle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comparing Aes operation result with expected result */
    if (memcmp ((void *)&aesResult[0], (void *)&gCryptoAesCcm128CipherText[0], 16) == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
        if(status == DTHE_AES_RETURN_SUCCESS)
        {
            status = memcmp ((void *)&gCryptoAesCCM128Tag[0], (void *)&gAesCmac128ExpectedTag[0], 4);
            if ((memcmp ((void *)&gCryptoAesCCM128Tag[0], (void *)&gAesCmac128ExpectedTag[0], 6)) == 0)
            {
                status = DTHE_AES_RETURN_SUCCESS;
            }
            else
            {
                status = DTHE_AES_RETURN_FAILURE;
            }
        }
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        /* Initialize the encryption parameters */
        aesParams.algoType          = DTHE_AES_CCM_MODE;
        aesParams.opType            = DTHE_AES_DECRYPT;
        aesParams.ptrKey            = (uint32_t*)&gCryptoAesCcm128Key[0];
        aesParams.ptrIV             = (uint32_t*)&gCryptoAesCcm128Nonce[0];
        aesParams.dataLenBytes      = 16U;
        aesParams.useKEKMode        = FALSE;
        aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
        aesParams.aadLength         = 16;
        aesParams.ptrAAD            = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCcm128AAD[0]);
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;
        aesParams.streamState       = DTHE_AES_STREAM_INIT;
        aesParams.streamSize        = 0U;
        aesParams.ccmL              = app_aes_ccm_getCCM_L(8);/* Pass nonce lenght in bytes*/
        aesParams.ccmM              = app_aes_ccm_getCCM_M(6);/* Pass tag lenght in bytes*/

        /* Decryption */
        status = DTHE_AES_execute(handle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        /* Initialize the encryption parameters */
        aesParams.ptrPlainTextData  = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCCM128DecData[0]);
        aesParams.ptrEncryptedData  = (uint32_t*)SOC_virtToPhy((void *)&aesResult[0]);
        aesParams.aadLength         = 0;
        aesParams.ptrTag            = (uint32_t*)&gCryptoAesCCM128Tag[0];
        aesParams.streamState       = DTHE_AES_STREAM_FINISH;
        aesParams.streamSize        = 16U;
        aesParams.dataLenBytes      = 16U;

        /* Decryption */
        status = DTHE_AES_execute(handle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(handle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(handle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Comparing Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCCM128DecData[0], (void *)&gCryptoAesCcm128PlainText[0], APP_CRYPTO_AES_CCM_128_INOUT_LENGTH) == 0)
    {
        if(status == DTHE_AES_RETURN_SUCCESS)
        {
            if (memcmp ((void *)&gCryptoAesCCM128Tag[0], (void *)&gAesCmac128ExpectedTag[0], 4) == 0)
            {
                status = DTHE_AES_RETURN_SUCCESS;
                DebugP_log("[CRYPTO] DTHE AES CCM-128 example completed!!\r\n");
                DebugP_log("All tests have passed!!\r\n");
            }
            else
            {
                status = DTHE_AES_RETURN_FAILURE;
            }
        }
    }
    else
    {
        DebugP_log("[CRYPTO] DTHE AES CCM-128 example failed!!\r\n");
    }

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return;
}
uint32_t app_aes_ccm_getCCM_L(uint16_t noOnceLenght)
{
    uint32_t ccmL;
    switch(noOnceLenght)
    {
    case 7:
        ccmL = 0x7;
        break;
    case 8:
        ccmL = 0x6;
        break;
    case 9:
        ccmL = 0x5;
        break;
    case 10:
        ccmL = 0x4;
        break;
    case 11:
        ccmL = 0x3;
        break;
    case 12:
        ccmL = 0x2;
        break;
    case 13:
        ccmL = 0x1;
        break;
    case 14:
        ccmL = 0x0;
        break;
    default:
        ccmL=-1;
        break;
    }
    return ccmL;
}

uint32_t app_aes_ccm_getCCM_M(uint16_t tagLenght)
{
    uint32_t ccmM;
    switch(tagLenght)
    {
    case 4:
        ccmM = 0x1;
        break;
    case 6:
        ccmM = 0x2;
        break;
    case 8:
        ccmM = 0x3;
        break;
    case 10:
        ccmM = 0x4;
        break;
    case 12:
        ccmM = 0x5;
        break;
    case 14:
        ccmM = 0x6;
        break;
    case 16:
        ccmM = 0x7;
        break;
    default:
        ccmM=-1;
        break;
    }
    return ccmM;
}
