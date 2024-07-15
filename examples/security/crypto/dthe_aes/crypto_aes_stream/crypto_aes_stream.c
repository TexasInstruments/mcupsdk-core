/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <kernel/dpl/SystemP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Input length*/
#define APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH          (32*1024)
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

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                     (1U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* All zero buffer used while processing key1 and key2*/
uint8_t gCryptoAesCmacZerosInput[APP_CRYPTO_AES_BLOCK_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* input buffer for Cmac*/
uint8_t gCryptoAesCmacInputBuffer[APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH] =
{
    0x00U
};

/*Expected Cmac results*/
static const uint8_t gAesCmac256ExpectedResult[16] = {
    0x37, 0x74, 0x09, 0xb0, 0xdb, 0x75, 0xc3, 0x26,
    0xc0, 0x6d, 0xd3, 0x1f, 0x5c, 0x08, 0x15, 0xf5
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesCmac256Key[APP_CRYPTO_AES_CMAC_256_MAXKEY_LENGTH] =
{
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
};

uint8_t gCryptoCmacConst_Rb[16] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87
};

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

uint32_t gCmac_unprocessed_len = 0;

uint8_t gCmac_unprocessed_block[APP_CRYPTO_AES_BLOCK_LENGTH];

/* Cmac output buf */
uint8_t gCryptoAesCmac256ResultBuf[APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH] __attribute__ ((aligned (128)));

DTHE_Handle         gAesHandle;

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
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t app_aes_ecb_256(uint8_t *input,uint8_t *key ,uint8_t inputLen, uint8_t *output);
void app_aes_cmac_256( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac);
void app_aes_cmac_256_stream_aligned( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac);
int32_t app_aes_cmac_dthe_256_stream_equal_aligned(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint8_t *tag);

void app_aes_cmac_256_stream_unaligned( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac);
int32_t app_aes_cmac_dthe_256_stream_start(uint8_t *key, uint8_t *k1, uint8_t *k2);
int32_t app_aes_cmac_dthe_256_stream_update(uint8_t **input, uint32_t ilen);
int32_t app_aes_cmac_dthe_256_stream_finish(uint8_t *tag);

void app_padding ( uint8_t *lastb, uint8_t *pad, int32_t length );
void app_xor_128(uint8_t *a, uint8_t *b, uint8_t *out);
static void app_leftShift(uint8_t *input, uint8_t *output);
int32_t app_cmacGenSubKeys(Crypto_Params *params);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void crypto_aes_stream_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_AES_Return_t   status;
    int32_t             testErrCont = 0;

    DebugP_log("[CRYPTO] AES CMAC-256 Streaming Example started ...\r\n");

    /* opens DTHe driver */
    gAesHandle = DTHE_open(0);
    DebugP_assert(gAesHandle != NULL);

    memset(gCryptoAesCmacInputBuffer,0x61,APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH);

    app_aes_cmac_256_stream_aligned(gCryptoAesCmac256Key, gCryptoAesCmacInputBuffer, APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH, gCryptoAesCmac256ResultBuf);

    /* Comparing final AES CMAC result with expected test results*/
    if(memcmp(gCryptoAesCmac256ResultBuf, gAesCmac256ExpectedResult, APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 Streaming Aligned Data Failed!!\r\n");
        testErrCont ++;
    }
    else
    {
        DebugP_log("[CRYPTO] AES CMAC-256 Streaming Aligned Data Passed!!\r\n");
    }

    app_aes_cmac_256_stream_unaligned(gCryptoAesCmac256Key, gCryptoAesCmacInputBuffer, APP_CRYPTO_AES_CMAC_256_INPUT_LENGTH, gCryptoAesCmac256ResultBuf);

    /* Comparing final AES CMAC result with expected test results*/
    if(memcmp(gCryptoAesCmac256ResultBuf, gAesCmac256ExpectedResult, APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 Streaming Unaligned Data Failed!!\r\n");
        testErrCont ++;
    }
    else
    {
        DebugP_log("[CRYPTO] AES CMAC-256 Streaming Unaligned Data Passed!!\r\n");
    }

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

    if(testErrCont == 0)
    {
        DebugP_log("[CRYPTO] AES CMAC-256 example completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void app_aes_cmac_256_stream_aligned( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac)
{
    Crypto_Params       params;
    int32_t             status;

    /* Subkeys Key1 and Key2 are derived from K through the subkey generation algorithm */
    /* AES-128 with key is applied to an all-zero input block. */
    status = app_aes_ecb_256(gCryptoAesCmacZerosInput, key, APP_CRYPTO_AES_BLOCK_LENGTH, params.aesWithKeyAppliedToZeroInput);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);
    /* Subkey generation algorithm */
    status = app_cmacGenSubKeys(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    app_aes_cmac_dthe_256_stream_equal_aligned(input, key, &params.key1[0], &params.key2[0], length, mac);

    return;
}

void app_aes_cmac_256_stream_unaligned( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac)
{
    Crypto_Params       params;
    int32_t             status;

    /* Subkeys Key1 and Key2 are derived from K through the subkey generation algorithm */
    /* AES-128 with key is applied to an all-zero input block. */
    status = app_aes_ecb_256(gCryptoAesCmacZerosInput, key, APP_CRYPTO_AES_BLOCK_LENGTH, params.aesWithKeyAppliedToZeroInput);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);
    /* Subkey generation algorithm */
    status = app_cmacGenSubKeys(&params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* opens aes driver */
    status = DTHE_AES_open(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Start CALL*/
    app_aes_cmac_dthe_256_stream_start(key, &params.key1[0], &params.key2[0]);

    /* Update CALL : First Stream of */
    app_aes_cmac_dthe_256_stream_update(&input, (4096U+10U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (16384U+20U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (8192U-15U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (4096U-15U));

    /* Finish CALL */
    app_aes_cmac_dthe_256_stream_finish(mac);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return;
}

int32_t app_aes_cmac_dthe_256_stream_start(uint8_t *key, uint8_t *k1, uint8_t *k2)
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
    aesParams.ptrPlainTextData  = (uint32_t*)NULL;
    aesParams.dataLenBytes      = 0U;
    aesParams.ptrTag            = (uint32_t*)NULL;
    aesParams.streamState       = DTHE_AES_STREAM_INIT;
    aesParams.streamSize        = 0U;

    /* Start Call */
    status = DTHE_AES_execute(gAesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

int32_t app_aes_cmac_dthe_256_stream_update(uint8_t **input, uint32_t ilen)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t n = 0;

    if((gCmac_unprocessed_len > 0) &&
	    (ilen > (APP_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len )))
    {
    	/* fill the unprocessed buffer */
        memcpy(&gCmac_unprocessed_block[gCmac_unprocessed_len],
                *input,
                APP_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len );

        /* Initialize the AES Parameters: */
        (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the encryption parameters: */
        aesParams.algoType          = DTHE_AES_CMAC_MODE;
        aesParams.opType            = DTHE_AES_ENCRYPT;
        aesParams.ptrKey            = (uint32_t*)NULL;
        aesParams.ptrKey1           = (uint32_t*)NULL;
        aesParams.ptrKey2           = (uint32_t*)NULL;
        aesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        aesParams.streamSize        = APP_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(gAesHandle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += APP_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len;
        ilen -= APP_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len;
        gCmac_unprocessed_len = 0;
    }

    /* n is the number of blocks including any final partial block */
    n = ( ilen + APP_CRYPTO_AES_BLOCK_LENGTH - 1 ) / APP_CRYPTO_AES_BLOCK_LENGTH;

    if(n>1)
    {
        /* Initialize the AES Parameters: */
        (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the encryption parameters: */
        aesParams.algoType          = DTHE_AES_CMAC_MODE;
        aesParams.opType            = DTHE_AES_ENCRYPT;
        aesParams.ptrKey            = (uint32_t*)NULL;
        aesParams.ptrKey1           = (uint32_t*)NULL;
        aesParams.ptrKey2           = (uint32_t*)NULL;
        aesParams.ptrPlainTextData  = (uint32_t*)(*input);
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        aesParams.streamSize        = (n-1)*APP_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(gAesHandle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += (n-1)*APP_CRYPTO_AES_BLOCK_LENGTH;
		ilen -= (n-1)*APP_CRYPTO_AES_BLOCK_LENGTH;
    }

    /* If there is data left over that wasn't aligned to a block */
    if( ilen > 0 )
    {
        memcpy( &gCmac_unprocessed_block[gCmac_unprocessed_len],
                *input, ilen );
        gCmac_unprocessed_len += ilen;
    }

    return (status);
}

int32_t app_aes_cmac_dthe_256_stream_finish(uint8_t *tag)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters: */
    aesParams.algoType          = DTHE_AES_CMAC_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)NULL;
    aesParams.ptrKey1           = (uint32_t*)NULL;
    aesParams.ptrKey2           = (uint32_t*)NULL;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
    aesParams.dataLenBytes      = 0U;
    aesParams.ptrTag            = (uint32_t*)tag;
    aesParams.streamState       = DTHE_AES_STREAM_FINISH;
    aesParams.streamSize        = gCmac_unprocessed_len;

    /* Start Call */
    status = DTHE_AES_execute(gAesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memset(gCmac_unprocessed_block, 0U, APP_CRYPTO_AES_BLOCK_LENGTH);
    gCmac_unprocessed_len = 0;

    return (status);
}

int32_t app_aes_cmac_dthe_256_stream_equal_aligned(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint8_t *tag)
{
    int32_t             status;

    /* opens aes driver */
    status = DTHE_AES_open(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Start CALL*/
    app_aes_cmac_dthe_256_stream_start(key, &k1[0], &k2[0]);

    /* Update CALL : First Stream of */
    app_aes_cmac_dthe_256_stream_update(&input, (4096U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (16384U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (8192U));

    /* Update CALL*/
    app_aes_cmac_dthe_256_stream_update(&input, (4096U));

    /* Finish CALL */
    app_aes_cmac_dthe_256_stream_finish(tag);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return status;
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
