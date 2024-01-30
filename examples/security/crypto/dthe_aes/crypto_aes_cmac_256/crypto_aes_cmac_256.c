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

/* This example demonstrates the DTHE AES CMAC 256. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/crypto.h>
#include <security/crypto/dthe/dthe.h>
#include <security/crypto/dthe/dthe_aes.h>
#include <kernel/dpl/SystemP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Input length*/
#define APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH          (64U)
/* Aes block length*/
#define APP_CRYPTO_AES_BLOCK_LENGTH                   (16U)
/* Aes Cmac output length*/
#define APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH             (16U)
/* Aes 256 key length*/
#define APP_CRYPTO_AES_CMAC_256_MAXKEY_LENGTH         (32U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                        (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                    (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                    (0xCE005000U)

/* All zero buffer used while processing key1 and key2*/
uint8_t gCryptoAesCmacZerosInput[APP_CRYPTO_AES_BLOCK_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* input buffer for Cmac*/
uint8_t gCryptoAesCmacInputBuffer[APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH] =
{
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
    0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
    0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
    0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
    0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
    0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
    0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
    0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10
};

/*Expected Cmac results*/
static const uint8_t gAesCmac256ExpectedResult[3][16] = {
    {
        /* Example #1 */
        0x28, 0xa7, 0x02, 0x3f, 0x45, 0x2e, 0x8f, 0x82,
        0xbd, 0x4b, 0xf2, 0x8d, 0x8c, 0x37, 0xc3, 0x5c
    },
    {
        /* Example #2 */
        0xaa, 0xf3, 0xd8, 0xf1, 0xde, 0x56, 0x40, 0xc2,
        0x32, 0xf5, 0xb1, 0x69, 0xb9, 0xc9, 0x11, 0xe6
    },
    {
        /* Example #3 */
        0xe1, 0x99, 0x21, 0x90, 0x54, 0x9f, 0x6e, 0xd5,
        0x69, 0x6a, 0x2c, 0x05, 0x6c, 0x31, 0x54, 0x10
    }
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesCmac256Key[APP_CRYPTO_AES_CMAC_256_MAXKEY_LENGTH] =
{
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
};

/* Different input sizes */
int32_t gCryptoCmac256DiffInputSizes[3] =
{
    16, 40, 64
};

uint8_t gCryptoCmacConst_Rb[16] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87
};

/* Cmac output buf */
uint8_t gCryptoAesCmac256ResultBuf[APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH] __attribute__ ((aligned (128)));
int32_t app_aes_ecb_256(uint8_t *input,uint8_t *key ,uint8_t inputLen, uint8_t *output);
void app_aes_cmac_256( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac);
int32_t app_aes_cmac_dthe_256(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint8_t *tag);
void app_padding ( uint8_t *lastb, uint8_t *pad, int32_t length );
void app_xor_128(uint8_t *a, uint8_t *b, uint8_t *out);
static void app_leftShift(uint8_t *input, uint8_t *output);
int32_t app_cmacGenSubKeys(Crypto_Params *params);

DTHE_Handle         gAesHandle;

void crypto_aes_cmac_256_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_AES_Return_t   status;
    int32_t             testErrCont = 0;

    DebugP_log("[CRYPTO] AES CMAC-256 example started ...\r\n");

    /* opens DTHe driver */
    gAesHandle = DTHE_open(0);
    DebugP_assert(gAesHandle != NULL);

    /* Aes Cmac with 16 bytes input */
    app_aes_cmac_256(gCryptoAesCmac256Key, gCryptoAesCmacInputBuffer, gCryptoCmac256DiffInputSizes[0], gCryptoAesCmac256ResultBuf);

    /* Comparing final AES CMAC result with expected test results*/
    if(memcmp(gCryptoAesCmac256ResultBuf, gAesCmac256ExpectedResult[0], APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 example1 failed!!\r\n");
        testErrCont ++;
    }

    /* Aes Cmac with 20 bytes input */
    app_aes_cmac_256(gCryptoAesCmac256Key, gCryptoAesCmacInputBuffer, gCryptoCmac256DiffInputSizes[1], gCryptoAesCmac256ResultBuf);

    /* Comparing final AES CMAC result with expected test results*/
    if(memcmp(gCryptoAesCmac256ResultBuf, gAesCmac256ExpectedResult[1], APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 example2 failed!!\r\n");
        testErrCont ++;
    }

    /* Aes Cmac with 64 bytes input */
    app_aes_cmac_256(gCryptoAesCmac256Key, gCryptoAesCmacInputBuffer, gCryptoCmac256DiffInputSizes[2], gCryptoAesCmac256ResultBuf);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(gAesHandle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comparing final AES CMAC result with expected test results*/
    if(memcmp(gCryptoAesCmac256ResultBuf, gAesCmac256ExpectedResult[2], APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 example3 failed!!\r\n");
        testErrCont ++;
    }

    if(testErrCont == 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void app_aes_cmac_256( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac)
{
    Crypto_Params       params;
    int32_t             status;

    /* Subkeys Key1 and Key2 are derived from K through the subkey generation algorithm */
    /* AES-256 with key is applied to an all-zero input block. */
    status = app_aes_ecb_256(gCryptoAesCmacZerosInput, key, APP_CRYPTO_AES_BLOCK_LENGTH, params.aesWithKeyAppliedToZeroInput);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Subkey generation algorithm */
    status = app_cmacGenSubKeys(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    app_aes_cmac_dthe_256(input, key, &params.key1[0], &params.key2[0], length, mac);

    return;
}

int32_t app_aes_cmac_dthe_256(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint8_t *tag)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters: */
    aesParams.algoType          = DTHE_AES_CMAC_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&key[0];
    aesParams.ptrKey1           = (uint32_t*)&k1[0];
    aesParams.ptrKey2           = (uint32_t*)&k2[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&input[0];
    aesParams.dataLenBytes      = inputLen;
    aesParams.ptrTag            = (uint32_t*)&tag[0];

    /* opens aes driver */
    status = DTHE_AES_open(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(gAesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

int32_t app_aes_ecb_256(uint8_t *input, uint8_t *key, uint8_t inputLen, uint8_t *output)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters: */
    aesParams.algoType          = DTHE_AES_ECB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)&key[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrPlainTextData  = (uint32_t*)&input[0];
    aesParams.dataLenBytes      = inputLen;
    aesParams.ptrEncryptedData  = (uint32_t*)&output[0];

    /* opens aes driver */
    status = DTHE_AES_open(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(gAesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

void app_xor_128(uint8_t *a, uint8_t *b, uint8_t *out)
{
    int32_t i;
    for (i=0;i<APP_CRYPTO_AES_BLOCK_LENGTH; i++)
    {
        out[i] = a[i] ^ b[i];
    }

    return;
}

void app_padding ( uint8_t *lastb, uint8_t *pad, int32_t length )
{
    int32_t j;

    /* original last block */
    for ( j=0; j<APP_CRYPTO_AES_BLOCK_LENGTH; j++ )
    {
        if ( j < length )
        {
            pad[j] = lastb[j];
        }
        else if ( j == length )
        {
            pad[j] = 0x80;
        }
        else
        {
            pad[j] = 0x00;
        }
    }

    return;
}

int32_t app_cmacGenSubKeys(Crypto_Params *params)
{
    uint8_t             tmp[CRYPTO_AES_BLOCK_LENGTH];
    int32_t             status = SystemP_SUCCESS;

    if(NULL == params)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if((params->aesWithKeyAppliedToZeroInput[0] &0x80) == 0)
        {
            app_leftShift(params->aesWithKeyAppliedToZeroInput, params->key1);
        }
        else
        {
            app_leftShift(params->aesWithKeyAppliedToZeroInput, tmp);
            app_xor_128(tmp, gCryptoCmacConst_Rb, params->key1);
        }

        if((params->key1[0] &0x80)==0)
        {
            app_leftShift(params->key1, params->key2);
        }
        else
        {
            app_leftShift(params->key1, tmp);
            app_xor_128(tmp, gCryptoCmacConst_Rb, params->key2);
        }
    }

    return (status);
}

static void app_leftShift(uint8_t *input, uint8_t *output)
{
    int32_t             i;
    uint8_t             overflow = 0;

    for( i=(CRYPTO_AES_BLOCK_LENGTH - 1); i>=0; i-- )
    {
        output[i] = input[i] << 1;
        output[i] |= overflow;
        overflow = (input[i] & 0x80)?1:0;
    }
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