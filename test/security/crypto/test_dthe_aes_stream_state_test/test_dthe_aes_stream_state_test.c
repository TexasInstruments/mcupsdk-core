/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This test demonstrates the HW implementation of AES stream State test */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <unity.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <string.h>
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_aes.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Aes block length*/
#define TEST_CRYPTO_AES_BLOCK_LENGTH                  (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH         (32U)
/* Aes 512B buf length*/
#define TEST_CRYPTO_AES_TEST_512B_BUF_LEN             (512U)
/* Aes 256B buf length*/
#define TEST_CRYPTO_AES_TEST_256B_BUF_LEN             (256U)
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
DTHE_AES_Params     gAesParams;

/** cmac unprocessed data length */
uint32_t gCmac_unprocessed_len = 0;
/** cmac unprocessed buffer holder */
uint8_t gCmac_unprocessed_block[TEST_CRYPTO_AES_BLOCK_LENGTH];

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesEcb256Key[APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

/* input buffer for encryption or decryption */
uint8_t     gCryptoAesEcbInputBuf[TEST_CRYPTO_AES_TEST_512B_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Encryption output buf */
uint8_t     gCryptoAesEcbEncResultBuf[TEST_CRYPTO_AES_TEST_512B_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Decryption output buf */
uint8_t     gCryptoAesEcbDecResultBuf[TEST_CRYPTO_AES_TEST_512B_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

DTHE_Handle         aesHandle;

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

int32_t app_aes_ecb_dthe_stream_start(uint8_t *key, uint32_t keyLen, uint32_t encOrDec);
int32_t app_aes_ecb_dthe_stream_update(uint8_t **input, uint8_t **output, uint32_t ilen, uint32_t encOrDec);
int32_t app_aes_ecb_dthe_stream_finish(void);

/** Negative sequence of api's, Test cases with start after start call */
void test_aes_state_start_to_start(void *args);
/** Positive sequence of api's, Test cases with init to finish call */
void test_aes_state_start_to_finish(void *args);
/** Negative sequence of api's, Test cases with start after update call */
void test_aes_state_update_to_start(void *args);
/** Negative sequence of api's, Test cases with update after finish call */
void test_aes_state_finish_to_update(void *args);
/** Negative sequence of api's, Test cases with finish after finish call */
void test_aes_state_finish_to_finish(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    DTHE_AES_Return_t    status;

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    RUN_TEST(test_aes_state_start_to_start,     (uint32_t)12917, (void *)NULL);
    RUN_TEST(test_aes_state_start_to_finish,   (uint32_t)12918, (void *)NULL);
    RUN_TEST(test_aes_state_update_to_start,   (uint32_t)12919, (void *)NULL);
    RUN_TEST(test_aes_state_finish_to_update, (uint32_t)12920, (void *)NULL);
    RUN_TEST(test_aes_state_finish_to_finish, (uint32_t)12921, (void *)NULL);

    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(aesHandle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
    UNITY_END();
    Board_driversClose();
    Drivers_close();
}

/** Unity framework required functions */
void setUp(void)
{
}

void tearDown(void)
{
}

/** Negative sequence of api's, Test cases with start after start call */
void test_aes_state_start_to_start(void *args)
{
    DTHE_AES_Return_t   status;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)gCryptoAesEcb256Key;
    gAesParams.keyLen            = DTHE_AES_KEY_256_SIZE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    /* Start CALL*/
    status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
        TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_FAILURE, status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
}

/** Positive sequence of api's, Test cases with init to finish call */
void test_aes_state_start_to_finish(void *args)
{
    DTHE_AES_Return_t   status;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)gCryptoAesEcb256Key;
    gAesParams.keyLen            = DTHE_AES_KEY_256_SIZE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    uint8_t *localInputBuff = &gCryptoAesEcbInputBuf[0];
    uint8_t *localOutputBuff1 = &gCryptoAesEcbEncResultBuf[0];
    uint8_t *localOutputBuff2 = &gCryptoAesEcbEncResultBuf[0];

    /* fill the unprocessed buffer */
    memcpy(&gCmac_unprocessed_block[0],
            gCryptoAesEcbInputBuf,
            TEST_CRYPTO_AES_BLOCK_LENGTH);

    app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff1, TEST_CRYPTO_AES_BLOCK_LENGTH, DTHE_AES_ENCRYPT);
    
    gAesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
    gAesParams.ptrEncryptedData  = (uint32_t*)localOutputBuff1;
    app_aes_ecb_dthe_stream_finish();

    /* fill the unprocessed buffer */
    memcpy(&gCmac_unprocessed_block[0],
            gCryptoAesEcbInputBuf,
            TEST_CRYPTO_AES_BLOCK_LENGTH);

    app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff2, TEST_CRYPTO_AES_BLOCK_LENGTH, DTHE_AES_ENCRYPT);
    gAesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
    gAesParams.ptrEncryptedData  = (uint32_t*)localOutputBuff2;
    app_aes_ecb_dthe_stream_finish();

    /* comparing result with expected test results */
    status = memcmp(localOutputBuff1, localOutputBuff2, TEST_CRYPTO_AES_BLOCK_LENGTH);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
}

/** Negative sequence of api's, Test cases with init after update call */
void test_aes_state_update_to_start(void *args)
{

    DTHE_AES_Return_t   status;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)gCryptoAesEcb256Key;
    gAesParams.keyLen            = DTHE_AES_KEY_256_SIZE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    uint8_t *localInputBuff = &gCryptoAesEcbInputBuf[0];
    uint8_t *localOutputBuff = &gCryptoAesEcbEncResultBuf[0];

    /* Start CALL*/
    status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);        
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);        
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
        TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_FAILURE, status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
}

/** Negative sequence of api's, Test cases with update after finish call */
void test_aes_state_finish_to_update(void *args)
{
    DTHE_AES_Return_t   status;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)gCryptoAesEcb256Key;
    gAesParams.keyLen            = DTHE_AES_KEY_256_SIZE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    uint8_t *localInputBuff = &gCryptoAesEcbInputBuf[0];
    uint8_t *localOutputBuff = &gCryptoAesEcbEncResultBuf[0];

    /* Start CALL*/
    status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);        
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);        
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_finish();       
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);
        TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_FAILURE, status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
}

/** Negative sequence of api's, Test cases with finish after finish call */
void test_aes_state_finish_to_finish(void *args)
{
    DTHE_AES_Return_t   status;

    memset(gCryptoAesEcbInputBuf,0x00,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbEncResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);
    memset(gCryptoAesEcbDecResultBuf,0xFF,TEST_CRYPTO_AES_TEST_512B_BUF_LEN);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)gCryptoAesEcb256Key;
    gAesParams.keyLen            = DTHE_AES_KEY_256_SIZE;

    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    uint8_t *localInputBuff = &gCryptoAesEcbInputBuf[0];
    uint8_t *localOutputBuff = &gCryptoAesEcbEncResultBuf[0];

    /* Start CALL*/
    status = app_aes_ecb_dthe_stream_start(gCryptoAesEcb256Key, DTHE_AES_KEY_256_SIZE, DTHE_AES_ENCRYPT);
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);        
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_update(&localInputBuff, &localOutputBuff, TEST_CRYPTO_AES_TEST_256B_BUF_LEN, DTHE_AES_ENCRYPT);    
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_finish();
    }
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        status = app_aes_ecb_dthe_stream_finish();
        TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_FAILURE, status);
    }

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);
}

int32_t app_aes_ecb_dthe_stream_finish(void)
{
    DTHE_AES_Return_t   status;

    /* Initialize the encryption parameters: */
    gAesParams.dataLenBytes      = 0U;
    gAesParams.streamState       = DTHE_AES_STREAM_FINISH;
    gAesParams.streamSize        = gCmac_unprocessed_len;

    /* Start Call */
    status = DTHE_AES_execute(aesHandle, &gAesParams);

    memset(gCmac_unprocessed_block, 0U, TEST_CRYPTO_AES_BLOCK_LENGTH);
    gCmac_unprocessed_len = 0;

    return (status);
}

int32_t app_aes_ecb_dthe_stream_update(uint8_t **input, uint8_t **output, uint32_t ilen, uint32_t encOrDec)
{
    DTHE_AES_Return_t   status;
    uint32_t n = 0;

    if((gCmac_unprocessed_len > 0) &&
        (ilen > (TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len )))
    {
        /* fill the unprocessed buffer */
        memcpy(&gCmac_unprocessed_block[gCmac_unprocessed_len],
                *input,
                TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len );

        /* Initialize the encryption parameters: */
        gAesParams.useKEKMode        = FALSE;
        if (encOrDec == DTHE_AES_ENCRYPT)
        {
            gAesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
            gAesParams.ptrEncryptedData  = (uint32_t*)(*output);
        }
        else
        {
            gAesParams.ptrEncryptedData  = (uint32_t*)&gCmac_unprocessed_block[0U];
            gAesParams.ptrPlainTextData  = (uint32_t*)(*output);   
        }
        gAesParams.dataLenBytes      = 0U;
        gAesParams.ptrTag            = (uint32_t*)NULL;
        gAesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        gAesParams.streamSize        = TEST_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(aesHandle, &gAesParams);

        *input += (TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len);
        *output += ((TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len) +gCmac_unprocessed_len);
        ilen -= (TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len);
        gCmac_unprocessed_len = 0;
    }

    /* n is the number of blocks including any final partial block */
    n = ( ilen + TEST_CRYPTO_AES_BLOCK_LENGTH - 1 ) / TEST_CRYPTO_AES_BLOCK_LENGTH;

    if(n>1)
    {
        if (encOrDec == DTHE_AES_ENCRYPT)
        {
            /* Initialize the encryption parameters: */
            gAesParams.ptrPlainTextData  = (uint32_t*)(*input);
            gAesParams.ptrEncryptedData  = (uint32_t*)(*output);
        }
        else
        {
            /* Initialize the encryption parameters: */
            gAesParams.ptrPlainTextData  = (uint32_t*)(*output);
            gAesParams.ptrEncryptedData  = (uint32_t*)(*input);
        }
        
        gAesParams.dataLenBytes      = 0U;
        gAesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        gAesParams.streamSize        = (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(aesHandle, &gAesParams);

        *input += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
        *output += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;

        ilen -= (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
    }

    /* If there is data left over that wasn't aligned to a block */
    if( ilen > 0 )
    {
        memcpy( &gCmac_unprocessed_block[gCmac_unprocessed_len],
            *input, ilen );
        gCmac_unprocessed_len += ilen;
        if(ilen == TEST_CRYPTO_AES_BLOCK_LENGTH)
        {
            *input += ilen;
        }
    }

    return (status);
}
int32_t app_aes_ecb_dthe_stream_start(uint8_t *key, uint32_t keyLen, uint32_t encOrDec)
{
    DTHE_AES_Return_t   status;

    /* Initialize the encryption parameters: */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.opType            = encOrDec;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)&key[0];
    gAesParams.keyLen            = keyLen;
    gAesParams.ptrPlainTextData  = (uint32_t*)NULL;
    gAesParams.dataLenBytes      = 0U;
    gAesParams.ptrTag            = (uint32_t*)NULL;
    gAesParams.streamState       = DTHE_AES_STREAM_INIT;
    gAesParams.streamSize        = 0U;

    /* Start Call */
    status = DTHE_AES_execute(aesHandle, &gAesParams);

    return (status);
}
