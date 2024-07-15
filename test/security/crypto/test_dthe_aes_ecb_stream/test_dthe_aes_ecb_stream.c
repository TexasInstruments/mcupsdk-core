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

/* This test demonstrates the HW implementation of AES ECB Stream */

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
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_aes.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Aes block length*/
#define TEST_CRYPTO_AES_BLOCK_LENGTH                  (16U)
/* Supported Operations */
#define APP_OPERATION_ENCRYPT                         (1U)
#define APP_OPERATION_DECRYPT                         (2U)

/* Supported Key length*/
#define APP_CRYPTO_AES_ECB_128                        (128U)
#define APP_CRYPTO_AES_ECB_256                        (256U)

/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_128_MAXKEY_LENGTH          (16U)
/* Aes max key length*/
#define APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH         (32U)
/* Aes 32k buf length*/
#define TEST_CRYPTO_AES_TEST_32K_BUF_LEN              (32768U)
/* Aes 16k buf length*/
#define TEST_CRYPTO_AES_TEST_16K_BUF_LEN              (16384U)
/* Aes 8k buf length*/
#define TEST_CRYPTO_AES_TEST_8K_BUF_LEN               (8192U)
/* Aes 4k buf length*/
#define TEST_CRYPTO_AES_TEST_4K_BUF_LEN               (4096U)
/* Aes 2k buf length*/
#define TEST_CRYPTO_AES_TEST_2K_BUF_LEN               (2048U)
/* Aes 1k buf length*/
#define TEST_CRYPTO_AES_TEST_1K_BUF_LEN               (1024U)
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
/* Total number of Test case*/
#define TEST_CRYPTO_AES_TEST_CASES_COUNT              (112U)

/* aligned buffer size */
#define TEST_ALIGNED_BUFF_SIZE                        (512U)
/* number of tests*/
#define TEST_COUNT                                    (14U)

/* EDMA config instance */
#define CONFIG_EDMA_NUM_INSTANCES                     (1U)

typedef struct
{
    uint16_t key;
    char operation[20];
    uint16_t dataSize;
    uint32_t streamSize;
    double performance;
}App_benchmark;

typedef struct testParams_t
{
    uint32_t testInputLength;
    uint32_t testId;
    uint32_t keyLen;
    uint32_t keyLenForKpiData;
    uint8_t *key;
}testParams;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint32_t localTrackBufNumber = 0;
/** ecb unprocessed data length */
uint32_t gEcb_unprocessed_len = 0;
/** ecb unprocessed buffer holder */
uint8_t gEcb_unprocessed_block[TEST_CRYPTO_AES_BLOCK_LENGTH];

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesEcb128Key[APP_CRYPTO_AES_ECB_128_MAXKEY_LENGTH] =
{
    0x93, 0x28, 0x67, 0x64, 0xA8, 0x51, 0x46, 0x73,
    0x0E, 0x64, 0x18, 0x88, 0xDB, 0x34, 0xEB, 0x47
};

/* The AES encryption algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit, 192-bit, or 256-bit keys */
static uint8_t gCryptoAesEcb256Key[APP_CRYPTO_AES_ECB_256_MAX_KEY_LENGTH] =
{
    0x33, 0xA3, 0x66, 0x46, 0xFE, 0x56, 0xF7, 0x0D,
    0xC0, 0xC5, 0x1A, 0x31, 0x17, 0xE6, 0x39, 0xF1,
    0x82, 0xDE, 0xF8, 0xCA, 0xB5, 0xC0, 0x66, 0x71,
    0xEE, 0xA0, 0x40, 0x7C, 0x48, 0xA9, 0xC7, 0x57
};

uint32_t gInputStreamSizes[] = {
                            TEST_CRYPTO_AES_TEST_256B_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_512B_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_1K_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_2K_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_4K_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_8K_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_16K_BUF_LEN,
                            TEST_CRYPTO_AES_TEST_32K_BUF_LEN,
                        };

/* input buffer for encryption or decryption */
uint8_t     gCryptoAesEcbInputBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Encryption output buf */
uint8_t     gCryptoAesEcbEncResultBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));
/* Decryption output buf */
uint8_t     gCryptoAesEcbDecResultBuf[TEST_CRYPTO_AES_TEST_32K_BUF_LEN] __attribute__((aligned(128), section(".bss.filebuf")));

/* Edma handler*/
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

DTHE_Handle         aesHandle;

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

/* Local test functions */
int32_t app_aes_ecb_dthe_stream_start(uint8_t *key, uint32_t keyLen, uint32_t encOrDec);
int32_t app_aes_ecb_dthe_stream_update(uint8_t **input, uint8_t **output, uint32_t ilen, uint32_t encOrDec);
int32_t app_aes_ecb_dthe_stream_finish(void);
int32_t test_aes_ecb_dthe(uint8_t *input, uint8_t *output, uint8_t *key, uint32_t keySize, uint32_t inputLen,  uint32_t encOrDec, uint32_t streamSize);
static void test_aes_ecb_stream(void *args);
void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation, uint32_t streamSize);
static const char *bytesToString(uint64_t bytes);
void App_printPerformanceLogs(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    DTHE_AES_Return_t    status;
    uint32_t loopForTestCount = 0;

    testParams testRunParams[] = {
        {TEST_CRYPTO_AES_TEST_512B_BUF_LEN,12863, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_1K_BUF_LEN,  12862, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_2K_BUF_LEN,  12861, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_4K_BUF_LEN,  12860, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_8K_BUF_LEN,  12859, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_16K_BUF_LEN, 12858, DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_32K_BUF_LEN, 12857,  DTHE_AES_KEY_256_SIZE, APP_CRYPTO_AES_ECB_256, gCryptoAesEcb256Key},
        {TEST_CRYPTO_AES_TEST_512B_BUF_LEN,12870, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_1K_BUF_LEN,  12869, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_2K_BUF_LEN,  12868, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_4K_BUF_LEN,  12867, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_8K_BUF_LEN,  12866, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_16K_BUF_LEN, 12865, DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},
        {TEST_CRYPTO_AES_TEST_32K_BUF_LEN, 12864,  DTHE_AES_KEY_128_SIZE, APP_CRYPTO_AES_ECB_128, gCryptoAesEcb128Key},

    };

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    /** ecb 128 and 256 with different input buffers*/
    for (loopForTestCount = 0; loopForTestCount < TEST_COUNT; loopForTestCount++)
    {
        if(( loopForTestCount == 0 ) || ( loopForTestCount == TEST_COUNT/2))
        {
            DebugP_log("[CRYPTO] AES ECB-%d Hw tests started ...\r\n", testRunParams[loopForTestCount].keyLenForKpiData);
        }
        RUN_TEST(test_aes_ecb_stream, (uint32_t)testRunParams[loopForTestCount].testId, (void *)&testRunParams[loopForTestCount]);
    }

    App_printPerformanceLogs();
    /* Closing DTHE driver */
    if (DTHE_RETURN_SUCCESS == DTHE_close(aesHandle))
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

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

DTHE_AES_Params     gAesParams;

void test_aes_ecb_stream(void *args)
{
    DTHE_AES_Return_t   status;
    uint32_t            t1, t2;
    uint32_t loopForMultipleStreamCases = 0;

    testParams *ptrTestPrms =  args;

    memset(gCryptoAesEcbInputBuf,0x00,ptrTestPrms->testInputLength);
    memset(gCryptoAesEcbEncResultBuf,0xFF,ptrTestPrms->testInputLength);
    memset(gCryptoAesEcbDecResultBuf,0xFF,ptrTestPrms->testInputLength);

    /* Initialize the AES Parameters */
    (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the encryption parameters */
    gAesParams.algoType          = DTHE_AES_ECB_MODE;
    gAesParams.useKEKMode        = FALSE;
    gAesParams.ptrKey            = (uint32_t*)ptrTestPrms->key;
    gAesParams.keyLen            = ptrTestPrms->keyLen;

    for (loopForMultipleStreamCases = 0; (gInputStreamSizes[loopForMultipleStreamCases] <= ((ptrTestPrms->testInputLength)/2)); loopForMultipleStreamCases++)
    {
        CycleCounterP_reset();
        t1 = CycleCounterP_getCount32();
        /* opens aes driver */
        status = DTHE_AES_open(aesHandle);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        /* Encryption */
        test_aes_ecb_dthe(gCryptoAesEcbInputBuf, gCryptoAesEcbEncResultBuf, ptrTestPrms->key, ptrTestPrms->keyLen, ptrTestPrms->testInputLength, DTHE_AES_ENCRYPT, gInputStreamSizes[loopForMultipleStreamCases]);
        localTrackBufNumber = 0;
        /* Closing aes driver */
        status = DTHE_AES_close(aesHandle);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        t2 = CycleCounterP_getCount32();
        App_fillPerformanceResults(t1, t2, ptrTestPrms->testInputLength, ptrTestPrms->keyLenForKpiData, APP_OPERATION_ENCRYPT, gInputStreamSizes[loopForMultipleStreamCases]);

        /* Initialize the AES Parameters */
        (void)memset ((void *)&gAesParams, 0, sizeof(DTHE_AES_Params));

        /* Initialize the encryption parameters */
        gAesParams.algoType          = DTHE_AES_ECB_MODE;
        gAesParams.useKEKMode        = FALSE;
        gAesParams.ptrKey            = (uint32_t*)ptrTestPrms->key;
        gAesParams.keyLen            = ptrTestPrms->keyLen;

        CycleCounterP_reset();
        t1 = CycleCounterP_getCount32();

        /* opens aes driver */
        status = DTHE_AES_open(aesHandle);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        /* Decryption */
        test_aes_ecb_dthe(gCryptoAesEcbEncResultBuf, gCryptoAesEcbDecResultBuf, ptrTestPrms->key, ptrTestPrms->keyLen, ptrTestPrms->testInputLength, DTHE_AES_DECRYPT, gInputStreamSizes[loopForMultipleStreamCases]);
        localTrackBufNumber =0;

        /* Closing aes driver */
        status = DTHE_AES_close(aesHandle);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        t2 = CycleCounterP_getCount32();
        App_fillPerformanceResults(t1, t2, ptrTestPrms->testInputLength, ptrTestPrms->keyLenForKpiData, APP_OPERATION_DECRYPT, gInputStreamSizes[loopForMultipleStreamCases]);

        /* comparing result with expected test results */
        status = memcmp(gCryptoAesEcbDecResultBuf, gCryptoAesEcbInputBuf, ptrTestPrms->testInputLength);
        TEST_ASSERT_EQUAL_UINT32(SystemP_SUCCESS, status);
    }
}

int32_t test_aes_ecb_dthe(uint8_t *input, uint8_t *output, uint8_t *key, uint32_t keySize, uint32_t inputLen,  uint32_t encOrDec, uint32_t streamSize)
{
    DTHE_AES_Return_t   status;
    uint32_t loopCount = 0;
    /* opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Start CALL*/
    app_aes_ecb_dthe_stream_start(key, keySize, encOrDec);

    for (loopCount = 0; loopCount < (inputLen/streamSize); loopCount++)
    {
        /* Update CALL*/
        app_aes_ecb_dthe_stream_update(&input, &output, streamSize, encOrDec);
    }


    if (encOrDec == DTHE_AES_ENCRYPT)
    {
        gAesParams.ptrPlainTextData  = (uint32_t*)&gEcb_unprocessed_block[0U];
        gAesParams.ptrEncryptedData  = (uint32_t*)output;
    }
    else
    {
        gAesParams.ptrEncryptedData = (uint32_t*)&gEcb_unprocessed_block[0U];
        gAesParams.ptrPlainTextData = (uint32_t*)output;
    }

    /* Finish CALL */
    app_aes_ecb_dthe_stream_finish();

    /* Closing aes driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    return (status);
}

int32_t app_aes_ecb_dthe_stream_finish(void)
{
    DTHE_AES_Return_t   status;

    /* Initialize the encryption parameters: */
    gAesParams.dataLenBytes      = 0U;
    gAesParams.streamState       = DTHE_AES_STREAM_FINISH;
    gAesParams.streamSize        = gEcb_unprocessed_len;

    /* Start Call */
    status = DTHE_AES_execute(aesHandle, &gAesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memset(gEcb_unprocessed_block, 0U, TEST_CRYPTO_AES_BLOCK_LENGTH);
    localTrackBufNumber += gEcb_unprocessed_len;
    gEcb_unprocessed_len = 0;

    return (status);
}

int32_t app_aes_ecb_dthe_stream_update(uint8_t **input, uint8_t **output, uint32_t ilen, uint32_t encOrDec)
{
    DTHE_AES_Return_t   status;
    uint32_t n = 0;

    if((gEcb_unprocessed_len > 0) &&
        (ilen > (TEST_CRYPTO_AES_BLOCK_LENGTH - gEcb_unprocessed_len )))
    {
        /* fill the unprocessed buffer */
        memcpy(&gEcb_unprocessed_block[gEcb_unprocessed_len],
                *input,
                TEST_CRYPTO_AES_BLOCK_LENGTH - gEcb_unprocessed_len );

        /* Initialize the encryption parameters: */
        gAesParams.useKEKMode        = FALSE;
        if (encOrDec == DTHE_AES_ENCRYPT)
        {
            gAesParams.ptrPlainTextData  = (uint32_t*)&gEcb_unprocessed_block[0U];
            gAesParams.ptrEncryptedData  = (uint32_t*)(*output);
        }
        else
        {
            gAesParams.ptrEncryptedData  = (uint32_t*)&gEcb_unprocessed_block[0U];
            gAesParams.ptrPlainTextData  = (uint32_t*)(*output);   
        }
        gAesParams.dataLenBytes      = 0U;
        gAesParams.ptrTag            = (uint32_t*)NULL;
        gAesParams.streamState       = DTHE_AES_STREAM_UPDATE;
        gAesParams.streamSize        = TEST_CRYPTO_AES_BLOCK_LENGTH;

        /* Update Call */
        status = DTHE_AES_execute(aesHandle, &gAesParams);
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += (TEST_CRYPTO_AES_BLOCK_LENGTH - gEcb_unprocessed_len);
        *output += TEST_CRYPTO_AES_BLOCK_LENGTH;
        ilen -= (TEST_CRYPTO_AES_BLOCK_LENGTH - gEcb_unprocessed_len);
        gEcb_unprocessed_len = 0;
        localTrackBufNumber +=TEST_CRYPTO_AES_BLOCK_LENGTH;
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
        DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

        *input += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
        *output += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;

        ilen -= (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
        localTrackBufNumber += (n-1)*TEST_CRYPTO_AES_BLOCK_LENGTH;
    }

    /* If there is data left over that wasn't aligned to a block */
    if( ilen > 0 )
    {
        memcpy( &gEcb_unprocessed_block[gEcb_unprocessed_len],
            *input, ilen );
        gEcb_unprocessed_len += ilen;
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
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

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

void App_fillPerformanceResults(uint32_t t1, uint32_t t2, uint32_t numBytes, uint32_t key, uint32_t operation, uint32_t streamSize)
{
    uint32_t diffCnt = 0;
    double cpuClkMHz = 0;
    double throughputInMBps = 0;
    cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    diffCnt = (t2 - t1);
    throughputInMBps  = (numBytes * cpuClkMHz)/diffCnt;

    App_benchmark *table = &results[gCount++];
    table-> key = key;
    if(operation == APP_OPERATION_ENCRYPT)
    {
        strcpy(table->operation, "Encryption");
    }
    else
    {
        strcpy(table->operation, "Decryption");
    }
    table->dataSize = numBytes;
    table->performance = (double)(8 * throughputInMBps);
    table->streamSize = streamSize;
}

void App_printPerformanceLogs()
{
    double cpuClkMHz = SOC_getSelfCpuClk()/1000000;
    DebugP_log("BENCHMARK START - DTHE - AES - ECB \r\n");
    DebugP_log("- Software/Application used : test_dthe_aes_ecb_stream \r\n");
    DebugP_log("- Code Placement            : OCRAM \r\n");
    DebugP_log("- Data Placement            : OCRAM \r\n");
    DebugP_log("- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB\r\n");
    DebugP_log("- CPU with operating speed  : R5F with %dMHZ \r\n", (uint32_t)cpuClkMHz);
    DebugP_log("| Key Length | operation  | Size | Stream Size | Performance (Mbps) | \r\n");
    DebugP_log("|------------|------------|------|-------------|--------------------| \r\n");
    for( uint32_t i = 0; i < TEST_CRYPTO_AES_TEST_CASES_COUNT; i++)
    {
        DebugP_log("| %d | %s | %s | %d | %lf |\r\n", results[i].key,  \
                    results[i].operation, bytesToString(results[i].dataSize), results[i].streamSize, results[i].performance);
    }
    DebugP_log("BENCHMARK END\r\n");
}
