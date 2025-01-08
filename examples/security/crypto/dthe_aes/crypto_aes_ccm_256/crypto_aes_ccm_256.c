/*
 * TIFS-MCU Source File
 *
 * This example demonstrates the DTHE AES 256 CCM Encryption and Decryption authentication
 *
 * Copyright (C) 2022-24 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED 
 * Licensed under the TI Software License Agreement found in [as_installed]/license.txt
 */

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
#define APP_CRYPTO_AES_CCM_256_INOUT_LENGTH             (4U)
/* AES CCM IV length in bytes */
#define APP_CRYPTO_AES_CCM_256_NONCE_LENGTH_IN_BYTES       (4U)
/* AES CCM KEY length in bytes */
#define APP_CRYPTO_AES_CCM_256_KEY_LENGTH_IN_BYTES      (8U)
/* AES CCM KEY Catche alignment size */
#define APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT         (32U)
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
static uint32_t gCryptoAesCcm256PlainText[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT))) =
{
    0x8e3445a8, 0xf1b5c5c8, 0x760ef526, 0x1e1bfdfe 
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint32_t gCryptoAesCcm256Key[APP_CRYPTO_AES_CCM_256_KEY_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT))) =
{ 
    0xb21576fb, 0x1d89803d, 0x0b9870d4, 0xc88495c7,
    0xce64fbb2, 0x4d8f9760, 0x5ae4fc17, 0xb730e849 
};

/* Encrypted buffer of gCryptoAesCcm256PlainText */
static uint32_t gCryptoAesCcm256CipherText[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT))) =
{
    0x611288cc, 0x72faa7c6, 0x39176ab9, 0x7f276b17 
};

/* Nonce is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
/* First byte should be Nonce CCML value, app_aes_ccm_getCCM_L() for getting ccmL value*/
/* Example Nonce lengght is 8 bytes, ccmL = app_aes_ccm_getCCM_L(8), ccmL = 0x6 put a 0th index of nonce*/
static uint32_t gCryptoAesCcm256Nonce[APP_CRYPTO_AES_CCM_256_NONCE_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT))) =
{
    0xa3d1db02, 0xb7246063, 0x7dda02b4, 0x0000006f
};

/* Expected tag */
static uint32_t gAesCmac256ExpectedTag[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH] __attribute__ ((aligned (APP_CRYPTO_AES_CCM_256_CATCHE_ALIGNMENT))) =
{
    0x14e17234, 0xbe0c2c5f, 0x06496314, 0x23e4f02c
};

/* Tag holder */
static uint32_t gCryptoAesCCM256Tag[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH];
/* Decrypted Data holder */
static uint32_t gCryptoAesCCM256DecData[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH];

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
void crypto_aes_ccm_256_main(void *args)
{
    DTHE_AES_Return_t       status;
    DTHE_Handle             handle;
    DTHE_AES_Params         aesParams;
    uint32_t                aesResult[APP_CRYPTO_AES_CCM_256_INOUT_LENGTH];

    /* opens DTHe driver */
    handle = DTHE_open(0);
    DebugP_assert(handle != NULL);

    DebugP_log("[CRYPTO] DTHE AES CCM-256 example started ...\r\n");

    /* Initialize the AES Parameters */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResult[0], 0xFF, sizeof(aesResult));


    status = DTHE_AES_open(handle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CCM_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCcm256Key[0];
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCcm256Nonce[0];
    aesParams.dataLenBytes      = 16U;
    aesParams.useKEKMode        = FALSE;
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.aadLength         = 0;
    aesParams.ptrAAD            = (uint32_t*)NULL;    
    aesParams.ptrTag            = (uint32_t*)NULL;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;
    aesParams.streamState       = DTHE_AES_STREAM_INIT;
    aesParams.streamSize        = 0U;
    aesParams.ccmL              = app_aes_ccm_getCCM_L(12);/* Pass nonce lenght in bytes*/
    aesParams.ccmM              = app_aes_ccm_getCCM_M(16);/* Pass tag lenght in bytes*/

    /* Encryption */
    status = DTHE_AES_execute(handle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);


    /* Initialize the encryption parameters */
    aesParams.ptrPlainTextData  = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCcm256PlainText[0]);
    aesParams.ptrEncryptedData  = (uint32_t*)SOC_virtToPhy((void *)&aesResult[0]);
    aesParams.aadLength         = 0;
    aesParams.ptrTag            = (uint32_t*)&gCryptoAesCCM256Tag[0];
    aesParams.streamState       = DTHE_AES_STREAM_FINISH;
    aesParams.streamSize        = 16U;
    aesParams.dataLenBytes      = 16U;

    /* Sending plaintext */
    status = DTHE_AES_execute(handle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comparing Aes operation result with expected result */
    if (memcmp ((void *)&aesResult[0], (void *)&gCryptoAesCcm256CipherText[0], 16) == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
        if(status == DTHE_AES_RETURN_SUCCESS)
        {
            status = memcmp ((void *)&gCryptoAesCCM256Tag[0], (void *)&gAesCmac256ExpectedTag[0], 4);
            if ((memcmp ((void *)&gCryptoAesCCM256Tag[0], (void *)&gAesCmac256ExpectedTag[0], 6)) == 0)
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
        aesParams.ptrKey            = (uint32_t*)&gCryptoAesCcm256Key[0];
        aesParams.ptrIV             = (uint32_t*)&gCryptoAesCcm256Nonce[0];
        aesParams.dataLenBytes      = 16U;
        aesParams.useKEKMode        = FALSE;
        aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
        aesParams.aadLength         = 0;
        aesParams.ptrAAD            = (uint32_t*)NULL;    
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;
        aesParams.streamState       = DTHE_AES_STREAM_INIT;
        aesParams.streamSize        = 0U;
        aesParams.ccmL              = app_aes_ccm_getCCM_L(12);/* Pass nonce lenght in bytes*/
        aesParams.ccmM              = app_aes_ccm_getCCM_M(16);/* Pass tag lenght in bytes*/

        /* Decryption */
        status = DTHE_AES_execute(handle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        /* Initialize the encryption parameters */
        aesParams.ptrPlainTextData  = (uint32_t*)SOC_virtToPhy((void *)&gCryptoAesCCM256DecData[0]);
        aesParams.ptrEncryptedData  = (uint32_t*)SOC_virtToPhy((void *)&aesResult[0]);
        aesParams.aadLength         = 0;
        aesParams.ptrTag            = (uint32_t*)&gCryptoAesCCM256Tag[0];
        aesParams.streamState       = DTHE_AES_STREAM_FINISH;
        aesParams.streamSize        = 16U;
        aesParams.dataLenBytes      = 16U;

        /* Sending encrypetd text */
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
    if (memcmp ((void *)&gCryptoAesCCM256DecData[0], (void *)&gCryptoAesCcm256PlainText[0], APP_CRYPTO_AES_CCM_256_INOUT_LENGTH) == 0)
    {
        if(status == DTHE_AES_RETURN_SUCCESS)
        {
            if (memcmp ((void *)&gCryptoAesCCM256Tag[0], (void *)&gAesCmac256ExpectedTag[0], 16) == 0)
            {
                status = DTHE_AES_RETURN_SUCCESS;
                DebugP_log("[CRYPTO] DTHE AES CCM-256 example completed!!\r\n");
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
        DebugP_log("[CRYPTO] DTHE AES CCM-256 example failed!!\r\n");
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
