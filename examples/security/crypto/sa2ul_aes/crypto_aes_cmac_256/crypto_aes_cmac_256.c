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

/* This example demonstrates the AES CMAC 256. */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
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
        0x28, 0xa7, 0x02, 0x3f,     0x45, 0x2e, 0x8f, 0x82,
        0xbd, 0x4b, 0xf2, 0x8d,     0x8c, 0x37, 0xc3, 0x5c
    },
    {
        /* Example #2 */
        0x15, 0x67, 0x27, 0xdc,     0x08, 0x78, 0x94, 0x4a,
        0x02, 0x3c, 0x1f, 0xe0,     0x3b, 0xad, 0x6d, 0x93
    },
    {
        /* Example #3 */
        0xe1, 0x99, 0x21, 0x90,     0x54, 0x9f, 0x6e, 0xd5,
        0x69, 0x6a, 0x2c, 0x05,     0x6c, 0x31, 0x54, 0x10
    }
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesCmac256Key[APP_CRYPTO_AES_CMAC_256_MAXKEY_LENGTH] =
{
    0x60, 0x3d, 0xeb, 0x10,     0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0,     0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07,     0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3,     0x09, 0x14, 0xdf, 0xf4
};

int32_t gCryptoCmac256DiffInputSizes[3] =
{
    16, 20, 64
};

/* Intermediate buffer to calculate Cmac */
uint8_t gCryptoAesCmac_X[APP_CRYPTO_AES_BLOCK_LENGTH];

/* Intermediate buffer to calculate Cmac  */
uint8_t gCryptoAesCmac_Y[APP_CRYPTO_AES_BLOCK_LENGTH];

/* Cmac output buf */
uint8_t gCryptoAesCmac256ResultBuf[APP_CRYPTO_AES_CMAC_OUTPUT_LENGTH] __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));
/* Context memory */
static Crypto_Context gCryptoAesEcb256Context __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

/* Context Object */
SA2UL_ContextObject  gSa2ulCtxObj __attribute__ ((aligned (SA2UL_CACHELINE_ALIGNMENT)));

int32_t app_aes_ecb_256(uint8_t *input,uint8_t *key ,uint8_t inputLen, uint8_t *output);
void app_aes_cmac_256( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac);
void app_padding ( uint8_t *lastb, uint8_t *pad, int32_t length );
void app_xor_128(uint8_t *a, uint8_t *b, uint8_t *out);

void crypto_aes_cmac_256(void *args)
{
    Drivers_open();
    Board_driversOpen();

    Crypto_Handle       aesHandle;
    int32_t             status,testErrCont = 0;

    aesHandle = Crypto_open(&gCryptoAesEcb256Context);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] AES CMAC-256 example started ...\r\n");
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

    /* Close AES instance */
    status = Crypto_close(aesHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();
}

void app_aes_cmac_256( uint8_t *key, uint8_t *input, int32_t length, uint8_t *mac)
{
    int32_t             i, numOfRounds,flag;
    uint8_t             M_last[APP_CRYPTO_AES_BLOCK_LENGTH], padded[APP_CRYPTO_AES_BLOCK_LENGTH];
    int32_t             status;
    uint32_t            aesBlockLength = APP_CRYPTO_AES_BLOCK_LENGTH;
    Crypto_Params       params;

    /* Subkeys Key1 and Key2 are derived from K through the subkey generation algorithm */
    /* AES-256 with key is applied to an all-zero input block. */
    status = app_aes_ecb_256(gCryptoAesCmacZerosInput, key, APP_CRYPTO_AES_BLOCK_LENGTH, params.aesWithKeyAppliedToZeroInput);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Subkey generation algorithm */
    status = Crypto_cmacGenSubKeys(&params);
    DebugP_assert(SystemP_SUCCESS == status);
    
    /* Number of rounds */
    numOfRounds =  (length+15)/ aesBlockLength;
    
    if ( numOfRounds == 0 )
    {
        numOfRounds = 1;
        flag = 0;
    }
    else
    {
        if(( length % aesBlockLength ) == 0 )
        {/* last block is a complete block */
            flag = 1;
        } 
        else 
        { /* last block is not complete block */
            flag = 0;
        }
    }

    if( flag )
    { /* last block is complete block */
        app_xor_128(&input[aesBlockLength*(numOfRounds-1)],params.key1,M_last);
    } 
    else
    {
        app_padding(&input[aesBlockLength*(numOfRounds-1)], padded, length%aesBlockLength);
        app_xor_128(padded, params.key2, M_last);
    }

    memset(gCryptoAesCmac_X, 0x00, aesBlockLength);
    for ( i=0; i<numOfRounds-1; i++ )
    {
        /* Y := Mi (+) X  */
        app_xor_128(gCryptoAesCmac_X, &input[aesBlockLength*i], gCryptoAesCmac_Y);
        /* X := AES-256(KEY, Y); */
        app_aes_ecb_256(gCryptoAesCmac_Y, key, APP_CRYPTO_AES_BLOCK_LENGTH, gCryptoAesCmac_X);
    }

    app_xor_128(gCryptoAesCmac_X, M_last, gCryptoAesCmac_Y);
    app_aes_ecb_256(gCryptoAesCmac_Y, key, APP_CRYPTO_AES_BLOCK_LENGTH, gCryptoAesCmac_X);

    memcpy(mac, gCryptoAesCmac_X, aesBlockLength);
}

int32_t app_aes_ecb_256(uint8_t *input, uint8_t *key, uint8_t inputLen, uint8_t *output)
{
    int32_t             status;
    SA2UL_ContextParams ctxParams;

    /* Configure secure context */
    ctxParams.opType       = SA2UL_OP_ENC;
    ctxParams.encAlg       = SA2UL_ENC_ALG_AES;
    ctxParams.encMode      = SA2UL_ENC_MODE_ECB;
    ctxParams.encKeySize   = SA2UL_ENC_KEYSIZE_256;
    ctxParams.encDirection = SA2UL_ENC_DIR_ENCRYPT;
    memcpy( &ctxParams.key[0], &key[0], APP_CRYPTO_AES_CMAC_256_MAXKEY_LENGTH);
    ctxParams.inputLen = inputLen;
    gSa2ulCtxObj.totalLengthInBytes = inputLen;

    /* Function to configure secure context */
    status = SA2UL_contextAlloc(gCryptoAesEcb256Context.drvHandle, &gSa2ulCtxObj, &ctxParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Perform cache writeback */
    CacheP_wb(input, inputLen, CacheP_TYPE_ALLD);
    CacheP_inv(input, inputLen, CacheP_TYPE_ALLD);

    /* Encryption */
    /* Function to transfer and receive data buffer */
    status = SA2UL_contextProcess(&gSa2ulCtxObj,&input[0], inputLen, output);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Function to free secure context configuration*/
    status = SA2UL_contextFree(&gSa2ulCtxObj);
    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}
void app_xor_128(uint8_t *a, uint8_t *b, uint8_t *out)
{
    int32_t i;
      for (i=0;i<APP_CRYPTO_AES_BLOCK_LENGTH; i++)
      {
          out[i] = a[i] ^ b[i];
      }
}
void app_padding ( uint8_t *lastb, uint8_t *pad, int32_t length )
  {
      int32_t j;

      /* original last block */
      for ( j=0; j<APP_CRYPTO_AES_BLOCK_LENGTH; j++ ) {
          if ( j < length ) {
              pad[j] = lastb[j];
          } else if ( j == length ) {
              pad[j] = 0x80;
          } else {
              pad[j] = 0x00;
          }
      }
  }