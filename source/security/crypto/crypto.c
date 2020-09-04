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

/**
 *  \file   crypto.c
 *
 *  \brief  This file contains the implementation of crypto driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <kernel/dpl/SystemP.h>
#include <security/crypto.h>
#include <security/crypto/sa2ul/sa2ul.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Crypto_leftShift(uint8_t *input, uint8_t *output);
static void Crypto_xor_128(uint8_t *a, uint8_t *b, uint8_t *out);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gCryptoCmacConst_Rb[16] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Crypto_init(void)
{
    /* Nothing to do now */
    return;
}

void Crypto_deinit(void)
{
    /* Nothing to do now */
    return;
}

Crypto_Handle Crypto_open(Crypto_Context *ctx)
{
    int32_t status = SystemP_SUCCESS;
    SA2UL_Params prms;
    Crypto_Handle    handle = NULL;

    if((NULL == ctx))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        ctx->drvHandle = SA2UL_open(0,&prms);

        if(ctx->drvHandle == NULL)
        {
            status = SystemP_FAILURE;
        }
        if(status == SystemP_SUCCESS)
        {
            handle = (Crypto_Handle) ctx;
        }
    }
    return (handle);
}

int32_t Crypto_close(Crypto_Handle handle)
{
    Crypto_Context  *ctx = (Crypto_Context *) handle;

    if(NULL != ctx)
    {
        SA2UL_close(ctx->drvHandle);
        memset(ctx, 0, sizeof(Crypto_Context));
    }
    return (0);
}

int32_t Crypto_hmacSha(Crypto_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    int32_t i = 0;
    uint8_t hmacKey[CRYPTO_HMAC_SHA512_KEYLEN_BYTES];
    uint8_t tempOut[CRYPTO_HMAC_SHA512_KEYLEN_BYTES];
    uint8_t *pIpad, *pOpad;
    uint8_t  keybyte, maxKeyLengthInBytes, inputKeyLengthInBytes;

    if(NULL == params)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        switch(params->authMode)
        {
            case SA2UL_HASH_ALG_SHA1:
                maxKeyLengthInBytes  = CRYPTO_HMAC_SHA1_KEYLEN_BYTES;
                break;

            case SA2UL_HASH_ALG_SHA2_256:
                maxKeyLengthInBytes  = CRYPTO_HMAC_SHA256_KEYLEN_BYTES;
                break;

            case SA2UL_HASH_ALG_SHA2_512:
                maxKeyLengthInBytes  = CRYPTO_HMAC_SHA512_KEYLEN_BYTES;
                break;

            default:
                status = SystemP_FAILURE;
                break;
        }

        if(SystemP_SUCCESS == status)
        {
            inputKeyLengthInBytes = ((uint32_t)params->keySizeInBytes);
            /* Check for KeyLength */
            if (inputKeyLengthInBytes > maxKeyLengthInBytes)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Copy input Key */
                memcpy(hmacKey, (uint8_t *)&params->key, inputKeyLengthInBytes);
            }
        }
        if(SystemP_SUCCESS == status)
        {
            inputKeyLengthInBytes = ((uint32_t)params->keySizeInBytes);
            /* Check for KeyLength */
            if (inputKeyLengthInBytes > maxKeyLengthInBytes)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Copy input Key */
                memcpy(hmacKey, (uint8_t *)&params->key, inputKeyLengthInBytes);

                /* Initialize the remaining bytes to zero if there are remaining bytes*/
                if(maxKeyLengthInBytes != inputKeyLengthInBytes)
                {
                    memset(hmacKey + inputKeyLengthInBytes, 0, maxKeyLengthInBytes - inputKeyLengthInBytes);
                }
                else
                {
                    /* Do nothing */
                }

                /* Stack optimization resue hmackey and tempOut buffers */
                pIpad = hmacKey;
                pOpad = tempOut;

                /* Compute Inner/Outer Padding  */
                for (i = 0; i < maxKeyLengthInBytes; i++)
                {
                    keybyte = hmacKey[i];
                    pIpad[i] = keybyte ^ CRYPTO_HMAC_SHA_IPAD;
                    pOpad[i] = keybyte ^ CRYPTO_HMAC_SHA_OPAD;
                }
                memcpy(&params->iPad, pIpad, maxKeyLengthInBytes);
                memcpy(&params->oPad, pOpad, maxKeyLengthInBytes);
            }
        }
    }
    return (status);
}

int32_t Crypto_cmacGenSubKeys(Crypto_Params *params)
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
            Crypto_leftShift(params->aesWithKeyAppliedToZeroInput, params->key1);
        }
        else
        {
            Crypto_leftShift(params->aesWithKeyAppliedToZeroInput, tmp);
            Crypto_xor_128(tmp, gCryptoCmacConst_Rb, params->key1);
        }

        if((params->key1[0] &0x80)==0)
        {
            Crypto_leftShift(params->key1, params->key2);
        }
        else
        {
            Crypto_leftShift(params->key1, tmp);
            Crypto_xor_128(tmp, gCryptoCmacConst_Rb, params->key2);
        }
    }

    return (status);
}

static void Crypto_leftShift(uint8_t *input, uint8_t *output)
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

static void Crypto_xor_128(uint8_t *a, uint8_t *b, uint8_t *out)
{
    int32_t i;
    for (i=0; i < CRYPTO_AES_BLOCK_LENGTH; i++)
    {
        out[i] = a[i] ^ b[i];
    }
    return;
}