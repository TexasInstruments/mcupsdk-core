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

/* This test demonstrates the HW implementation of AES CMAC */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_aes.h>
#include <security/security_common/drivers/crypto/crypto.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Aes block length*/
#define TEST_CRYPTO_AES_BLOCK_LENGTH                            (16U)
/* Aes 256 key length*/
#define APP_CRYPTO_AES_CMAC_256_KEY_LENGTH                      (32U)
/* Aes 128 key length*/
#define APP_CRYPTO_AES_CMAC_128_KEY_LENGTH                      (16U)
/* Aes Cmac output length*/
#define TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH                      (16U)
/* 32k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_32K_BUF_LEN                   (32768U)
/* 16k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_16K_BUF_LEN                   (16384U)
/* 8k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_8K_BUF_LEN                    (8192U)
/* 4k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_4K_BUF_LEN                    (4096U)
/* 2k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_2K_BUF_LEN                    (2048U)
/* 1k buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_1K_BUF_LEN                    (1024U)
/* 512 bytes buf length*/
#define TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN                  (512U)
/* Buffer alignment*/
#define TEST_CRYPTO_AES_CMAC_BUF_ALIGNMENT                      (128U)
/* Aes 16 byte key length in bits*/
#define CRYPTO_KEY_LEN_128                                      (128U)
/* Aes 32 byte key length in bits*/
#define CRYPTO_KEY_LEN_256                                      (256U)
/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                                  (0xCE000810U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_AES_U_BASE                              (0xCE007000U)
/* DTHE Aes Public address */
#define CSL_DTHE_PUBLIC_SHA_U_BASE                              (0xCE005000U)
/* Total number of Test case*/
#define TEST_CRYPTO_AES_TEST_CASES_COUNT                        (14U)

/* number of test vectors*/
#define TEST_NUM_VECTORS                                        (7U)
/* unaligned buffer size */
#define TEST_UNALIGNED_BUFF_SIZE                                (520U)

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                     (1U)

typedef struct
{
    uint16_t key;
    uint16_t dataSize;
    double performance;
}App_benchmark;

typedef struct testParams_t
{
    uint32_t testInputLength;
    uint32_t testId;
    uint32_t keyLen;
    uint32_t keySize;
    uint8_t *key;
    uint8_t *ptrExpectedOutput;
}testParams;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** cmac unprocessed data length */
uint32_t gCmac_unprocessed_len = 0;
/** cmac unprocessed buffer holder */
uint8_t gCmac_unprocessed_block[TEST_CRYPTO_AES_BLOCK_LENGTH];

/** 32k Test input buffer for cmac operation */
uint8_t gCryptoCmacTestInputBuf[TEST_CRYPTO_AES_CMAC_TEST_32K_BUF_LEN] __attribute__((aligned(TEST_CRYPTO_AES_CMAC_BUF_ALIGNMENT), section(".bss.filebuf")));
/** Test output buffer for cmac operation  */
uint8_t gCryptoCmacTestOutputBuf[TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH] __attribute__((aligned(TEST_CRYPTO_AES_CMAC_BUF_ALIGNMENT), section(".bss.filebuf")));

/** CMAC-256 test vectors,this is expected hash for the gCryptoCmacTestInputBuf buffer */
uint8_t gCryptoAesCmac256_32KbTestSum[TEST_NUM_VECTORS][TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH] =
{
    {0x37, 0x74, 0x09, 0xb0, 0xdb, 0x75, 0xc3, 0x26, 0xc0, 0x6d, 0xd3, 0x1f, 0x5c, 0x08, 0x15, 0xf5},
    {0x4a, 0x7c, 0xfb, 0x8e, 0x50, 0x41, 0x99, 0x41, 0x82, 0x17, 0x7a, 0xd7, 0xf2, 0xe1, 0x61, 0xf4},
    {0x6c, 0x3e, 0x10, 0x3f, 0x82, 0x7a, 0xc2, 0xe7, 0x24, 0xfb, 0xa5, 0x4f, 0x5f, 0x65, 0x37, 0xe4},
    {0xb3, 0x1c, 0x6a, 0xc2, 0xd9, 0xdf, 0x59, 0x9f, 0x35, 0x55, 0x10, 0x63, 0xb6, 0x94, 0xe3, 0xbb},
    {0xab, 0x3f, 0x44, 0x5f, 0x0f, 0x1b, 0x4f, 0x0c, 0x9f, 0xf5, 0x7a, 0xc4, 0x69, 0xb0, 0x99, 0x3d},
    {0x0c, 0xb9, 0x23, 0x53, 0x28, 0x01, 0xb5, 0x19, 0x60, 0x53, 0xac, 0xf5, 0x2b, 0x50, 0xfd, 0x03},
    {0x2e, 0x27, 0x77, 0x2e, 0xa1, 0x0e, 0xaf, 0x47, 0xe7, 0x77, 0x04, 0xb7, 0xf5, 0xc6, 0x83, 0x9d}
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesCmac256Key[APP_CRYPTO_AES_CMAC_256_KEY_LENGTH] =
{
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
};
/** CMAC-128 test vectors,this is expected hash for the gCryptoCmacTestInputBuf buffer */
uint8_t gCryptoAesCmac128_32KbTestSum[TEST_NUM_VECTORS][TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH] =
{
    {0xd0, 0x0e, 0x93, 0x58, 0x75, 0x67, 0x76, 0xfe, 0x5b, 0x1b, 0x93, 0x2c, 0xbc, 0xd9, 0x78, 0x66},
    {0x6f, 0x29, 0x80, 0xe7, 0x60, 0x7d, 0xda, 0xd0, 0x38, 0x96, 0x1e, 0x92, 0xf9, 0x5d, 0xfe, 0xbb},
    {0x81, 0x67, 0xf2, 0x1d, 0xe2, 0x3f, 0xd9, 0x7a, 0x7b, 0xb6, 0x28, 0xc9, 0x60, 0xa3, 0x91, 0x64},
    {0x15, 0x3d, 0x4b, 0x18, 0x98, 0x16, 0x89, 0xab, 0x19, 0x3a, 0x45, 0x2e, 0xa7, 0xc1, 0x4b, 0x2c},
    {0x9a, 0xd2, 0x73, 0x7f, 0x7d, 0x47, 0x73, 0x61, 0x23, 0x75, 0x1a, 0xb9, 0x05, 0xff, 0xe4, 0x5a},
    {0xe1, 0x16, 0x8c, 0x83, 0x02, 0xdf, 0x6e, 0xce, 0x64, 0x2a, 0xb7, 0xc7, 0x0e, 0x0a, 0xe9, 0x79},
    {0x0e, 0xe2, 0x64, 0xfa, 0x1c, 0xe7, 0x22, 0x8c, 0x65, 0xec, 0x6d, 0xdf, 0x87, 0xd3, 0xbc, 0x97}
};
/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
uint8_t gCryptoAesCmac128Key[APP_CRYPTO_AES_CMAC_128_KEY_LENGTH] =
{
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

uint8_t gCryptoCmacConst_Rb[16] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87
};
/* All zero buffer used while processing key1 and key2*/
uint8_t gCryptoAesCmacZerosInput[TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

DTHE_Handle         gAesCmacHandle;

uint16_t gCount = 0;
App_benchmark results[TEST_CRYPTO_AES_TEST_CASES_COUNT];

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

int32_t app_aes_cmac_dthe_stream_start(uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t keySize);
int32_t app_aes_cmac_dthe_stream_update(uint8_t **input, uint32_t ilen, uint32_t keySize);
int32_t app_aes_cmac_dthe_stream_finish(uint8_t *tag, uint32_t keySize);
int32_t test_aes_ecb(uint8_t *input,uint8_t *key ,uint8_t inputLen, uint8_t *output, uint32_t keySize);
void test_aes_cmac_stream( uint8_t *key, uint8_t *input, int32_t length, uint32_t keySize, uint8_t *mac);
void test_xor_128(uint8_t *a, uint8_t *b, uint8_t *out);
static void test_leftShift(uint8_t *input, uint8_t *output);
void crypto_aes_cmac_stream_unaligned(void *args);
int32_t test_aes_cmac_dthe(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint32_t keySize, uint8_t *tag);
int32_t test_cmacGenSubKeys(Crypto_Params *params);

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key);
static const char *bytesToString(uint64_t bytes);
void App_printPerformanceLogs(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();

    uint32_t loopForTestCount = 0;

    testParams testRunParams[] = {
        {TEST_CRYPTO_AES_CMAC_TEST_32K_BUF_LEN, 12809, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[0]},
        {TEST_CRYPTO_AES_CMAC_TEST_16K_BUF_LEN, 12810, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[1]},
        {TEST_CRYPTO_AES_CMAC_TEST_8K_BUF_LEN,  12811, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[2]},
        {TEST_CRYPTO_AES_CMAC_TEST_4K_BUF_LEN,  12812, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[3]},
        {TEST_CRYPTO_AES_CMAC_TEST_2K_BUF_LEN,  12813, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[4]},
        {TEST_CRYPTO_AES_CMAC_TEST_1K_BUF_LEN,  12814, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[5]},
        {TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN,12815, CRYPTO_KEY_LEN_256, DTHE_AES_KEY_256_SIZE, gCryptoAesCmac256Key, gCryptoAesCmac256_32KbTestSum[6]},
        {TEST_CRYPTO_AES_CMAC_TEST_32K_BUF_LEN, 12816, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[0]},
        {TEST_CRYPTO_AES_CMAC_TEST_16K_BUF_LEN, 12817, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[1]},
        {TEST_CRYPTO_AES_CMAC_TEST_8K_BUF_LEN,  12818, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[2]},
        {TEST_CRYPTO_AES_CMAC_TEST_4K_BUF_LEN,  12819, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[3]},
        {TEST_CRYPTO_AES_CMAC_TEST_2K_BUF_LEN,  12820, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[4]},
        {TEST_CRYPTO_AES_CMAC_TEST_1K_BUF_LEN,  12821, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[5]},
        {TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN,12822, CRYPTO_KEY_LEN_128, DTHE_AES_KEY_128_SIZE, gCryptoAesCmac128Key, gCryptoAesCmac128_32KbTestSum[6]}
    };

    /** cmac 256 and 128 with different input buffers*/
    for (loopForTestCount = 0; loopForTestCount < TEST_CRYPTO_AES_TEST_CASES_COUNT ; loopForTestCount++)
    {
        RUN_TEST(crypto_aes_cmac_stream_unaligned, (uint32_t)testRunParams[loopForTestCount].testId, (void *)&testRunParams[loopForTestCount]);
    }

    App_printPerformanceLogs();
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

void crypto_aes_cmac_stream_unaligned(void *args)
{
    DTHE_AES_Return_t   status;
    uint32_t            t1, t2;
    testParams *ptrTestPrms =  args;

    DebugP_log("AES CMAC-%d with %db buffer example started ...\r\n", ptrTestPrms->keyLen, ptrTestPrms->testInputLength);
    
    memset(gCryptoCmacTestInputBuf,0x61,ptrTestPrms->testInputLength);

    /* opens DTHe driver */
    gAesCmacHandle = DTHE_open(0);
    DebugP_assert(gAesCmacHandle != NULL);

    CycleCounterP_reset();
    t1 = CycleCounterP_getCount32();

    /* Aes Cmac with 32K to 512 bytes input */
    test_aes_cmac_stream(ptrTestPrms->key, gCryptoCmacTestInputBuf, ptrTestPrms->testInputLength, ptrTestPrms->keySize, gCryptoCmacTestOutputBuf);


    t2 = CycleCounterP_getCount32();

    App_fillPerformanceResults(t1, t2, ptrTestPrms->testInputLength, ptrTestPrms->keyLen);

    if(memcmp(gCryptoCmacTestOutputBuf, ptrTestPrms->ptrExpectedOutput, TEST_CRYPTO_AES_CMAC_OUTPUT_LENGTH) != 0)
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    else
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    
    if (status == DTHE_AES_RETURN_SUCCESS)
    {
        /* Closing DTHE driver */
        if (DTHE_RETURN_SUCCESS == DTHE_close(gAesCmacHandle))
        {
            status = DTHE_AES_RETURN_SUCCESS;
        }
        else
        {
            status = DTHE_AES_RETURN_FAILURE;
        }        
    }
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    return;
}

void test_aes_cmac_stream( uint8_t *key, uint8_t *input, int32_t length, uint32_t keySize, uint8_t *mac)
{
    int32_t             status;
    Crypto_Params       params;

    /* Subkeys Key1 and Key2 are derived from K through the subkey generation algorithm */
    /* AES-256 with key is applied to an all-zero input block. */
    status = test_aes_ecb(gCryptoAesCmacZerosInput, key, TEST_CRYPTO_AES_BLOCK_LENGTH, params.aesWithKeyAppliedToZeroInput, keySize);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    /* Subkey generation algorithm */
    status = test_cmacGenSubKeys(&params);
    TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);

    test_aes_cmac_dthe(input, key, &params.key1[0], &params.key2[0], length, keySize, mac);

    return;
}

int32_t test_aes_ecb(uint8_t *input, uint8_t *key, uint8_t inputLen, uint8_t *output, uint32_t keySize)
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
    aesParams.keyLen            = keySize;
    aesParams.ptrPlainTextData  = (uint32_t*)&input[0];
    aesParams.dataLenBytes      = inputLen;
    aesParams.ptrEncryptedData  = (uint32_t*)&output[0];

    /* opens aes driver */
    status = DTHE_AES_open(gAesCmacHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    /* Encryption */
    status = DTHE_AES_execute(gAesCmacHandle, &aesParams);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesCmacHandle);
    TEST_ASSERT_EQUAL_UINT32(DTHE_AES_RETURN_SUCCESS, status);

    return (status);
}

void test_xor_128(uint8_t *a, uint8_t *b, uint8_t *out)
{
    int32_t i;
    for (i=0;i<TEST_CRYPTO_AES_BLOCK_LENGTH; i++)
    {
        out[i] = a[i] ^ b[i];
    }

    return;
}

int32_t test_cmacGenSubKeys(Crypto_Params *params)
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
            test_leftShift(params->aesWithKeyAppliedToZeroInput, params->key1);
        }
        else
        {
            test_leftShift(params->aesWithKeyAppliedToZeroInput, tmp);
            test_xor_128(tmp, gCryptoCmacConst_Rb, params->key1);
        }

        if((params->key1[0] &0x80)==0)
        {
            test_leftShift(params->key1, params->key2);
        }
        else
        {
            test_leftShift(params->key1, tmp);
            test_xor_128(tmp, gCryptoCmacConst_Rb, params->key2);
        }
    }

    return (status);
}

static void test_leftShift(uint8_t *input, uint8_t *output)
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

int32_t test_aes_cmac_dthe(uint8_t *input, uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t inputLen, uint32_t keySize, uint8_t *tag)
{
    DTHE_AES_Return_t   status;
    uint32_t loopCount = 0;
    /* opens aes driver */
    status = DTHE_AES_open(gAesCmacHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Start CALL*/
    app_aes_cmac_dthe_stream_start(key, &k1[0], &k2[0], keySize);


    for (loopCount = 0; loopCount < (inputLen/TEST_UNALIGNED_BUFF_SIZE); loopCount++)
    {
        if ( inputLen != TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN)
        {
            /* Update CALL*/
            app_aes_cmac_dthe_stream_update(&input, TEST_UNALIGNED_BUFF_SIZE, keySize);
        }
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_32K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (8U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_16K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (264U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_8K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (392U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_4K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (456U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_2K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (488U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_1K_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (504U), keySize);
    }
    if ( inputLen == TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN)
    {
        app_aes_cmac_dthe_stream_update(&input, (TEST_CRYPTO_AES_CMAC_TEST_512B_BUF_LEN), keySize);        
    }

    /* Finish CALL */
    app_aes_cmac_dthe_stream_finish(tag, keySize);

    /* Closing aes driver */
    status = DTHE_AES_close(gAesCmacHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

int32_t app_aes_cmac_dthe_stream_start(uint8_t *key, uint8_t *k1, uint8_t *k2, uint32_t keySize)
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
    aesParams.keyLen            = keySize;
    aesParams.ptrPlainTextData  = (uint32_t*)NULL;
    aesParams.dataLenBytes      = 0U;
    aesParams.ptrTag            = (uint32_t*)NULL;
    aesParams.streamState       = DTHE_AES_STREAM_INIT;
    aesParams.streamSize        = 0U;

    /* Start Call */
    status = DTHE_AES_execute(gAesCmacHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

int32_t app_aes_cmac_dthe_stream_update(uint8_t **input, uint32_t ilen, uint32_t keySize)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t n = 0;

    if((gCmac_unprocessed_len > 0) &&
        (ilen > (TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len )))
    {
        /* fill the unprocessed buffer */
        memcpy(&gCmac_unprocessed_block[gCmac_unprocessed_len],
                *input,
                TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len );

        /* Initialize the AES Parameters: */
        (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the encryption parameters: */
        aesParams.algoType          = DTHE_AES_CMAC_MODE;
        aesParams.opType            = DTHE_AES_ENCRYPT;
        aesParams.useKEKMode        = FALSE;
        aesParams.ptrKey            = (uint32_t*)NULL;
        aesParams.ptrKey1           = (uint32_t*)NULL;
        aesParams.ptrKey2           = (uint32_t*)NULL;
        aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
        aesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
        aesParams.dataLenBytes      = 0U;
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        aesParams.streamSize        = TEST_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(gAesCmacHandle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len;
        ilen -= TEST_CRYPTO_AES_BLOCK_LENGTH - gCmac_unprocessed_len;
        gCmac_unprocessed_len = 0;
    }

    /* n is the number of blocks including any final partial block */
    n = ( ilen + TEST_CRYPTO_AES_BLOCK_LENGTH - 1 ) / TEST_CRYPTO_AES_BLOCK_LENGTH;

    if(n>1)
    {
        /* Initialize the AES Parameters: */
        (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the encryption parameters: */
        aesParams.algoType          = DTHE_AES_CMAC_MODE;
        aesParams.opType            = DTHE_AES_ENCRYPT;
        aesParams.useKEKMode        = FALSE;
        aesParams.ptrKey            = (uint32_t*)NULL;
        aesParams.ptrKey1           = (uint32_t*)NULL;
        aesParams.ptrKey2           = (uint32_t*)NULL;
        aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
        aesParams.ptrPlainTextData  = (uint32_t*)(*input);
        aesParams.dataLenBytes      = 0U;
        aesParams.ptrTag            = (uint32_t*)NULL;
        aesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        aesParams.streamSize        = (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(gAesCmacHandle, &aesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
        ilen -= (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
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

int32_t app_aes_cmac_dthe_stream_finish(uint8_t *tag, uint32_t keySize)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters: */
    aesParams.algoType          = DTHE_AES_CMAC_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.useKEKMode        = FALSE;
    aesParams.ptrKey            = (uint32_t*)NULL;
    aesParams.ptrKey1           = (uint32_t*)NULL;
    aesParams.ptrKey2           = (uint32_t*)NULL;
    aesParams.keyLen            = keySize;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCmac_unprocessed_block[0U];
    aesParams.dataLenBytes      = 0U;
    aesParams.ptrTag            = (uint32_t*)tag;
    aesParams.streamState       = DTHE_AES_STREAM_FINISH;
    aesParams.streamSize        = gCmac_unprocessed_len;

    /* Start Call */
    status = DTHE_AES_execute(gAesCmacHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memset(gCmac_unprocessed_block, 0U, TEST_CRYPTO_AES_BLOCK_LENGTH);
    gCmac_unprocessed_len = 0;

    return (status);
}

static const char *bytesToString(uint64_t bytes)
{
    char *suffix[] = {"B", "KB", "MB", "GB", "TB"};
    char length = sizeof(suffix) / sizeof(suffix[0]);

    int i = 0;
    double dblBytes = bytes;

    if (bytes > 1024) {
        for (i = 0; (bytes / 1024) > 0 && i<length-1; i++, bytes /= 1024)
            dblBytes = bytes / 1024.0;
    }

    static char output[200];
    sprintf(output, "%  .02lf %s", dblBytes, suffix[i]);
    return output;
}

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key)
{
    uint32_t diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);
    throughputInMBps  = (numBytes * cpuClkMHz)/diffCnt;

    App_benchmark *table = &results[gCount++];
    table-> key = key;
    table->dataSize = numBytes;
    table->performance = (double)(8 * throughputInMBps);
}

void App_printPerformanceLogs()
{
    double cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    DebugP_log("BENCHMARK START - DTHE - AES - CMAC \r\n");
    DebugP_log("- Software/Application used : test_dthe_aes_cmac_stream_unaligned \r\n");
    DebugP_log("- Code Placement            : OCRAM \r\n");
    DebugP_log("- Data Placement            : OCRAM \r\n");
    DebugP_log("- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB\r\n");
    DebugP_log("- CPU with operating speed  : R5F with %dMHZ \r\n", (uint32_t)cpuClkMHz);
    DebugP_log("| Key Length | Size | Performance (Mbps) | \r\n");
    DebugP_log("|------------|------|--------------------| \r\n");
    for( uint32_t i = 0; i < TEST_CRYPTO_AES_TEST_CASES_COUNT; i++)
    {
        DebugP_log("| %d | %s | %lf |\r\n", results[i].key,  \
                    bytesToString(results[i].dataSize), results[i].performance);
    }
    DebugP_log("BENCHMARK END\r\n");
}
