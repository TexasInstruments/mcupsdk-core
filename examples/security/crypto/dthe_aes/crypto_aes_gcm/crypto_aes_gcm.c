/*
 * TIFS-MCU Source File
 *
 * This example demonstrates the DTHE AES 128, 192, 256 bit key GCM Encryption and Decryption.
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 * Licensed under the TI Software License Agreement found in [as_installed]/license.txt
 */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <security/security_common/drivers/crypto/dthe/dthe.h>
#include <security/security_common/drivers/crypto/dthe/dthe_aes.h>
#include <security/security_common/drivers/crypto/dthe/dma.h>
#include <security/security_common/drivers/crypto/dthe/dma/edma/dthe_edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Buffer Size*/
#define APP_CRYPTO_AES_GCM_MAX_BUFF_SIZE                 (128U)


#define APP_CRYPTO_AES_GCM_INOUT_LENGTH_2                (80U)

/* IV length*/
#define APP_CRYPTO_AES_GCM_IV_LENGTH_IN_BYTES            (16U)

/* Tag length*/
#define APP_CRYPTO_AES_GCM_TAG_LENGTH                    (16U)

/* AES GCM KEY Cache alignment size */
#define APP_CRYPTO_AES_GCM_CACHE_ALIGNMENT              (32U)

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

/*********************************************************************************************************************/
/* Test Vector 1  (Key Len 128 bits) */
static uint8_t gCryptoAesGcmKey1[] =
{
    0xfeU, 0xffU, 0xe9U, 0x92U, 0x86U, 0x65U, 0x73U, 0x1cU, 0x6dU, 0x6aU, 0x8fU, 0x94U, 0x67U, 0x30U, 0x83U, 0x08U,
};

static uint8_t gCryptoAesGcmIv1[] =
{
    0xcaU, 0xfeU, 0xbaU, 0xbeU, 0xfaU, 0xceU, 0xdbU, 0xadU, 0xdeU, 0xcaU, 0xf8U, 0x88U, 0x00U, 0x00U, 0x00U, 0x01,
};

static uint8_t gCryptoAesGcmHash1[] =
{
    0xb8U, 0x3bU, 0x53U, 0x37U, 0x08U, 0xbfU, 0x53U, 0x5dU, 0x0aU, 0xa6U, 0xe5U, 0x29U, 0x80U, 0xd5U, 0x3bU, 0x78U,
};


static uint8_t gCryptoAesGcmAADText1[] =
{
    0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU, 0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU,
    0xabU, 0xadU, 0xdaU, 0xd2U,
};


static uint8_t gCryptoAesGcmPlainText1[] =
{
    0xd9U, 0x31U, 0x32U, 0x25U, 0xf8U, 0x84U, 0x06U, 0xe5U, 0xa5U, 0x59U, 0x09U, 0xc5U, 0xafU, 0xf5U, 0x26U, 0x9aU,
    0x86U, 0xa7U, 0xa9U, 0x53U, 0x15U, 0x34U, 0xf7U, 0xdaU, 0x2eU, 0x4cU, 0x30U, 0x3dU, 0x8aU, 0x31U, 0x8aU, 0x72U,
    0x1cU, 0x3cU, 0x0cU, 0x95U, 0x95U, 0x68U, 0x09U, 0x53U, 0x2fU, 0xcfU, 0x0eU, 0x24U, 0x49U, 0xa6U, 0xb5U, 0x25U,
    0xb1U, 0x6aU, 0xedU, 0xf5U, 0xaaU, 0x0dU, 0xe6U, 0x57U, 0xbaU, 0x63U, 0x7bU, 0x39U,
};

static uint8_t gCryptoAesGcmEncryptedText1[] =
{
    0x42U, 0x83U, 0x1eU, 0xc2U, 0x21U, 0x77U, 0x74U, 0x24U, 0x4bU, 0x72U, 0x21U, 0xb7U, 0x84U, 0xd0U, 0xd4U, 0x9cU,
    0xe3U, 0xaaU, 0x21U, 0x2fU, 0x2cU, 0x02U, 0xa4U, 0xe0U, 0x35U, 0xc1U, 0x7eU, 0x23U, 0x29U, 0xacU, 0xa1U, 0x2eU,
    0x21U, 0xd5U, 0x14U, 0xb2U, 0x54U, 0x66U, 0x93U, 0x1cU, 0x7dU, 0x8fU, 0x6aU, 0x5aU, 0xacU, 0x84U, 0xaaU, 0x05U,
    0x1bU, 0xa3U, 0x0bU, 0x39U, 0x6aU, 0x0aU, 0xacU, 0x97U, 0x3dU, 0x58U, 0xe0U, 0x91U,
};

static uint8_t gCryptoAesGcmTag1[] =
{
    0x5bU, 0xc9U, 0x4fU, 0xbcU, 0x32U, 0x21U, 0xa5U, 0xdbU, 0x94U, 0xfaU, 0xe9U, 0x5aU, 0xe7U, 0x12U, 0x1aU, 0x47U,
};

/*********************************************************************************************************************/
/* Test Vector 2 (Key Len 192 bits)  */
static uint8_t gCryptoAesGcmKey2[] =
{
    0xfeU, 0xffU, 0xe9U, 0x92U, 0x86U, 0x65U, 0x73U, 0x1cU, 0x6dU, 0x6aU, 0x8fU, 0x94U, 0x67U, 0x30U, 0x83U, 0x08U,
    0xfeU, 0xffU, 0xe9U, 0x92U, 0x86U, 0x65U, 0x73U, 0x1cU
};

static uint8_t gCryptoAesGcmIv2[] =
{
    0xcaU, 0xfeU, 0xbaU, 0xbeU, 0xfaU, 0xceU, 0xdbU, 0xadU, 0xdeU, 0xcaU, 0xf8U, 0x88U, 0x00U, 0x00U, 0x00U, 0x01U,
};

static uint8_t gCryptoAesGcmHash2[] =
{
    0x46U, 0x69U, 0x23U, 0xecU, 0x9aU, 0xe6U, 0x82U, 0x21U, 0x4fU, 0x2cU, 0x08U, 0x2bU, 0xadU, 0xb3U, 0x92U, 0x49U,
};

static uint8_t gCryptoAesGcmAADText2[] =
{
    0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU, 0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU,
    0xabU, 0xadU, 0xdaU, 0xd2U,
};

static uint8_t gCryptoAesGcmPlainText2[] =
{
    0xd9U, 0x31U, 0x32U, 0x25U, 0xf8U, 0x84U, 0x06U, 0xe5U, 0xa5U, 0x59U, 0x09U, 0xc5U, 0xafU, 0xf5U, 0x26U, 0x9aU,
    0x86U, 0xa7U, 0xa9U, 0x53U, 0x15U, 0x34U, 0xf7U, 0xdaU, 0x2eU, 0x4cU, 0x30U, 0x3dU, 0x8aU, 0x31U, 0x8aU, 0x72U,
    0x1cU, 0x3cU, 0x0cU, 0x95U, 0x95U, 0x68U, 0x09U, 0x53U, 0x2fU, 0xcfU, 0x0eU, 0x24U, 0x49U, 0xa6U, 0xb5U, 0x25U,
    0xb1U, 0x6aU, 0xedU, 0xf5U, 0xaaU, 0x0dU, 0xe6U, 0x57U, 0xbaU, 0x63U, 0x7bU, 0x39U,
};

static uint8_t gCryptoAesGcmEncryptedText2[] =
{
    0x39U, 0x80U, 0xcaU, 0x0bU, 0x3cU, 0x00U, 0xe8U, 0x41U, 0xebU, 0x06U, 0xfaU, 0xc4U, 0x87U, 0x2aU, 0x27U, 0x57U,
    0x85U, 0x9eU, 0x1cU, 0xeaU, 0xa6U, 0xefU, 0xd9U, 0x84U, 0x62U, 0x85U, 0x93U, 0xb4U, 0x0cU, 0xa1U, 0xe1U, 0x9cU,
    0x7dU, 0x77U, 0x3dU, 0x00U, 0xc1U, 0x44U, 0xc5U, 0x25U, 0xacU, 0x61U, 0x9dU, 0x18U, 0xc8U, 0x4aU, 0x3fU, 0x47U,
    0x18U, 0xe2U, 0x44U, 0x8bU, 0x2fU, 0xe3U, 0x24U, 0xd9U, 0xccU, 0xdaU, 0x27U, 0x10U,
};

static uint8_t gCryptoAesGcmTag2[] =
{
    0x25U, 0x19U, 0x49U, 0x8eU, 0x80U, 0xf1U, 0x47U, 0x8fU, 0x37U, 0xbaU, 0x55U, 0xbdU, 0x6dU, 0x27U, 0x61U, 0x8cU,
};

/***********************************************************************************************************************/
/* Test Vector 3 (Key len 256 bits) */
static uint8_t gCryptoAesGcmKey3[] =
{
    0xfeU, 0xffU, 0xe9U, 0x92U, 0x86U, 0x65U, 0x73U, 0x1cU, 0x6dU, 0x6aU, 0x8fU, 0x94U, 0x67U, 0x30U, 0x83U, 0x08U,
    0xfeU, 0xffU, 0xe9U, 0x92U, 0x86U, 0x65U, 0x73U, 0x1cU, 0x6dU, 0x6aU, 0x8fU, 0x94U, 0x67U, 0x30U, 0x83U, 0x08U,
};

static uint8_t gCryptoAesGcmIv3[] =
{
    0xcaU, 0xfeU, 0xbaU, 0xbeU, 0xfaU, 0xceU, 0xdbU, 0xadU, 0xdeU, 0xcaU, 0xf8U, 0x88U, 0x00U, 0x00U, 0x00U, 0x01U,
};

static uint8_t gCryptoAesGcmHash3[] =
{
    0xacU, 0xbeU, 0xf2U, 0x05U, 0x79U, 0xb4U, 0xb8U, 0xebU, 0xceU, 0x88U, 0x9bU, 0xacU, 0x87U, 0x32U, 0xdaU, 0xd7U,
};

static uint8_t gCryptoAesGcmAADText3[] =
{
    0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU, 0xfeU, 0xedU, 0xfaU, 0xceU, 0xdeU, 0xadU, 0xbeU, 0xefU,
    0xabU, 0xadU, 0xdaU, 0xd2U,
};

static uint8_t gCryptoAesGcmPlainText3[] =
{
    0xd9U, 0x31U, 0x32U, 0x25U, 0xf8U, 0x84U, 0x06U, 0xe5U, 0xa5U, 0x59U, 0x09U, 0xc5U, 0xafU, 0xf5U, 0x26U, 0x9aU,
    0x86U, 0xa7U, 0xa9U, 0x53U, 0x15U, 0x34U, 0xf7U, 0xdaU, 0x2eU, 0x4cU, 0x30U, 0x3dU, 0x8aU, 0x31U, 0x8aU, 0x72U,
    0x1cU, 0x3cU, 0x0cU, 0x95U, 0x95U, 0x68U, 0x09U, 0x53U, 0x2fU, 0xcfU, 0x0eU, 0x24U, 0x49U, 0xa6U, 0xb5U, 0x25U,
    0xb1U, 0x6aU, 0xedU, 0xf5U, 0xaaU, 0x0dU, 0xe6U, 0x57U, 0xbaU, 0x63U, 0x7bU, 0x39U,
};

static uint8_t gCryptoAesGcmEncryptedText3[] =
{
    0x52U, 0x2dU, 0xc1U, 0xf0U, 0x99U, 0x56U, 0x7dU, 0x07U, 0xf4U, 0x7fU, 0x37U, 0xa3U, 0x2aU, 0x84U, 0x42U, 0x7dU,
    0x64U, 0x3aU, 0x8cU, 0xdcU, 0xbfU, 0xe5U, 0xc0U, 0xc9U, 0x75U, 0x98U, 0xa2U, 0xbdU, 0x25U, 0x55U, 0xd1U, 0xaaU,
    0x8cU, 0xb0U, 0x8eU, 0x48U, 0x59U, 0x0dU, 0xbbU, 0x3dU, 0xa7U, 0xb0U, 0x8bU, 0x10U, 0x56U, 0x82U, 0x88U, 0x38U,
    0xc5U, 0xf6U, 0x1eU, 0x63U, 0x93U, 0xbaU, 0x7aU, 0x0aU, 0xbcU,0xc9U, 0xf6U, 0x62U,
};

static uint8_t gCryptoAesGcmTag3[] =
{
    0x76U, 0xfcU, 0x6eU, 0xceU, 0x0fU, 0x4eU, 0x17U, 0x68U, 0xcdU, 0xdfU, 0x88U, 0x53U, 0xbbU, 0x2dU, 0x55U, 0x1bU,
};

/***********************************************************************************************************************/
/* Structure to hold input parameter for GCM example*/
struct crypto_aes_gcm_test_input
{
   uint8_t  testId;
   uint8_t  *plainText;
   uint8_t  *AADText;
   uint8_t  *encryptedText;
   uint8_t  *tag;
   uint8_t  *key;
   uint8_t  *Iv;
   uint32_t dataLenBytes;
   uint32_t aadLength;
   uint8_t  keySize;
   uint8_t  *ptrHashInput;
};

/* Internal Functions Declaration*/
static DTHE_AES_Return_t apps_crypto_aes_gcm_execute(DTHE_Handle aesHandle, struct crypto_aes_gcm_test_input inputParam, uint8_t gcmMode, int32_t *testStatus);

DTHE_Handle			gShaHandle = NULL;

void crypto_aes_gcm_main(void *args)
{
    DTHE_AES_Return_t   status;
    int32_t             testStatus = SystemP_SUCCESS;
    DTHE_Handle         aesHandle;
    struct crypto_aes_gcm_test_input inputParam;

    Drivers_open();
    Board_driversOpen();

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("Starting DTHE GCM Test Examples!\r\n");

    /* Test 1: GCM with 128 bit Key, Mode 3 (no precomputed hash)*/
    DebugP_log("\r\n[CRYPTO] Test 1 : GCM with 128-bit key, mode 3 (hash computed)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 1U;
    inputParam.plainText      = gCryptoAesGcmPlainText1;
    inputParam.AADText        = gCryptoAesGcmAADText1;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText1;
    inputParam.tag            = gCryptoAesGcmTag1;
    inputParam.key            = gCryptoAesGcmKey1;
    inputParam.Iv             = gCryptoAesGcmIv1;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText1);
    inputParam.aadLength      = sizeof(gCryptoAesGcmAADText1);
    inputParam.keySize        = DTHE_AES_KEY_128_SIZE;
    inputParam.ptrHashInput   = NULL;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_3, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Test 2: GCM with 192 bit Key, Mode 3*/
    DebugP_log("\r\n[CRYPTO] Test 2 : GCM with 192-bit key, mode 3 (hash computed)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 2U;
    inputParam.plainText      = gCryptoAesGcmPlainText2;
    inputParam.AADText        = gCryptoAesGcmAADText2;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText2;
    inputParam.tag            = gCryptoAesGcmTag2;
    inputParam.key            = gCryptoAesGcmKey2;
    inputParam.Iv             = gCryptoAesGcmIv2;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText2);
    inputParam.aadLength     = sizeof(gCryptoAesGcmAADText2);
    inputParam.keySize        = DTHE_AES_KEY_192_SIZE;
    inputParam.ptrHashInput   = NULL;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_3, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Test 3: GCM with 256 bit Key, Mode 3*/
    DebugP_log("\r\n[CRYPTO] Test 3 : GCM with 256-bit key, mode 3 (hash computed)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 3U;
    inputParam.plainText      = gCryptoAesGcmPlainText3;
    inputParam.AADText        = gCryptoAesGcmAADText3;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText3;
    inputParam.tag            = gCryptoAesGcmTag3;
    inputParam.key            = gCryptoAesGcmKey3;
    inputParam.Iv             = gCryptoAesGcmIv3;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText3);
    inputParam.aadLength     = sizeof(gCryptoAesGcmAADText3);
    inputParam.keySize        = DTHE_AES_KEY_256_SIZE;
    inputParam.ptrHashInput   = NULL;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_3, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Test 4: GCM with 128 bit Key, Mode 2*/
    DebugP_log("\r\n[CRYPTO] Test 4 : GCM with 128-bit key, mode 2 (hash pre-loaded)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 4U;
    inputParam.plainText      = gCryptoAesGcmPlainText1;
    inputParam.AADText        = gCryptoAesGcmAADText1;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText1;
    inputParam.tag            = gCryptoAesGcmTag1;
    inputParam.key            = gCryptoAesGcmKey1;
    inputParam.Iv             = gCryptoAesGcmIv1;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText1);
    inputParam.aadLength     = sizeof(gCryptoAesGcmAADText1);
    inputParam.keySize        = DTHE_AES_KEY_128_SIZE;
    inputParam.ptrHashInput   = gCryptoAesGcmHash1;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_2, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Test 5: GCM with 192 bit Key, Mode 2*/
    DebugP_log("\r\n[CRYPTO] Test 5 : GCM with 192-bit key, mode 2 (hash pre-loaded)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 5U;
    inputParam.plainText      = gCryptoAesGcmPlainText2;
    inputParam.AADText        = gCryptoAesGcmAADText2;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText2;
    inputParam.tag            = gCryptoAesGcmTag2;
    inputParam.key            = gCryptoAesGcmKey2;
    inputParam.Iv             = gCryptoAesGcmIv2;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText2);
    inputParam.aadLength     = sizeof(gCryptoAesGcmAADText2);
    inputParam.keySize        = DTHE_AES_KEY_192_SIZE;
    inputParam.ptrHashInput   = gCryptoAesGcmHash2;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_2, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Test 6: GCM with 256 bit Key, Mode 2*/
    DebugP_log("\r\n[CRYPTO] Test 6 : GCM with 256-bit key, mode 2 (hash pre-loaded)\r\n");
    memset(&inputParam,0,sizeof(inputParam));
    inputParam.testId         = 6U;
    inputParam.plainText      = gCryptoAesGcmPlainText3;
    inputParam.AADText        = gCryptoAesGcmAADText3;
    inputParam.encryptedText  = gCryptoAesGcmEncryptedText3;
    inputParam.tag            = gCryptoAesGcmTag3;
    inputParam.key            = gCryptoAesGcmKey3;
    inputParam.Iv             = gCryptoAesGcmIv3;
    inputParam.dataLenBytes   = sizeof(gCryptoAesGcmPlainText3);
    inputParam.aadLength     = sizeof(gCryptoAesGcmAADText3);
    inputParam.keySize        = DTHE_AES_KEY_256_SIZE;
    inputParam.ptrHashInput   = gCryptoAesGcmHash3;

    status =  apps_crypto_aes_gcm_execute(aesHandle, inputParam, DTHE_AES_GCM_MODE_2, &testStatus);

    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    (void)DTHE_close(aesHandle);

    if(testStatus == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have FAILED!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

/* Function to execute AES GCM Encryption-Decryption along with Authentication*/
static DTHE_AES_Return_t apps_crypto_aes_gcm_execute(DTHE_Handle aesHandle, struct crypto_aes_gcm_test_input inputParam, uint8_t gcmMode, int32_t *testStatus)
{
    DTHE_AES_Return_t   status;
    DTHE_AES_Params     aesParams;
    uint32_t            aesResultEnc[APP_CRYPTO_AES_GCM_MAX_BUFF_SIZE/4U];
    uint32_t            aesResultDec[APP_CRYPTO_AES_GCM_MAX_BUFF_SIZE/4U];
    uint32_t            aesGcmTagEnc[4U];
    uint32_t            aesGcmTagDec[4U];

    /*Initiate Encryption operation*/
    DebugP_log("[CRYPTO] Initiated DTHE AES GCM Encryption with TAG generation\r\n");

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResultEnc[0], 0xFF, sizeof(aesResultEnc));
    (void)memset ((void *)&aesResultDec[0], 0xFF, sizeof(aesResultDec));
    (void)memset ((void *)&aesGcmTagEnc[0], 0xFF, sizeof(aesGcmTagEnc));
    (void)memset ((void *)&aesGcmTagDec[0], 0xFF, sizeof(aesGcmTagDec));

    /* Initialize the Encryption parameters */
    aesParams.algoType          = DTHE_AES_GCM_MODE;
    aesParams.modeSelect        = gcmMode;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)inputParam.key;
    aesParams.ptrKey1           = (uint32_t*)inputParam.ptrHashInput;
    aesParams.keyLen            = inputParam.keySize;
    aesParams.ptrIV             = (uint32_t*)inputParam.Iv;
    aesParams.ptrEncryptedData  = (uint32_t*)(&aesResultEnc[0]);
    aesParams.dataLenBytes      = inputParam.dataLenBytes;
    aesParams.ptrPlainTextData  = (uint32_t*)(inputParam.plainText);
    aesParams.ptrAAD            = (uint32_t *)(inputParam.AADText);
    aesParams.aadLength         =  inputParam.aadLength;
    aesParams.ptrTag            = (uint32_t*)&aesGcmTagEnc[0];
    aesParams.useKEKMode        = FALSE;


    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Encryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)inputParam.encryptedText, (void *)&aesResultEnc[0], inputParam.dataLenBytes) == 0)
    {
        if(memcmp ((void *)inputParam.tag, (void *)&aesGcmTagEnc[0], 16) == 0)
        {
            *testStatus |= SystemP_SUCCESS;
        }
		else
		{
			*testStatus |= SystemP_FAILURE;
            DebugP_log("[CRYPTO] DTHE AES GCM Authentication Tag generation failed\r\n");
		}
    }
    else
    {
        *testStatus |= SystemP_FAILURE;
        DebugP_log("[CRYPTO] DTHE AES GCM Encryption failed\r\n");
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    if(*testStatus == SystemP_SUCCESS)
    {
        DebugP_log("[CRYPTO] DTHE AES GCM Encryption and Tag generation Success!\r\n");

    }

    /*Initiate Decryption operation*/
    DebugP_log("[CRYPTO] Initiated DTHE AES GCM Decryption with TAG verification\r\n");

    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the Decryption parameters */
    aesParams.algoType          = DTHE_AES_GCM_MODE;
    aesParams.modeSelect        = gcmMode;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)inputParam.key;
    aesParams.ptrKey1           = (uint32_t*)inputParam.ptrHashInput;
    aesParams.keyLen            = inputParam.keySize;
    aesParams.ptrIV             = (uint32_t*)inputParam.Iv;
    aesParams.ptrEncryptedData  = (uint32_t*)(inputParam.encryptedText);
    aesParams.dataLenBytes      = inputParam.dataLenBytes;
    aesParams.ptrPlainTextData  = (uint32_t*)(&aesResultDec[0]);
    aesParams.ptrAAD            = (uint32_t *)(inputParam.AADText);
    aesParams.aadLength         =  inputParam.aadLength;
    aesParams.ptrTag            = (uint32_t*)&aesGcmTagDec[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)inputParam.plainText, (void *)&aesResultDec[0], inputParam.dataLenBytes) == 0)
    {
        if(memcmp ((void *)inputParam.tag, (void *)&aesGcmTagDec[0], 16U) == 0)
        {
            *testStatus |= SystemP_SUCCESS;
        }
        else
		{
			*testStatus |= SystemP_FAILURE;
            DebugP_log("[CRYPTO] DTHE AES GCM Authentication Tag verification failed\r\n");
		}
    }
    else
    {
        *testStatus |= SystemP_FAILURE;
        DebugP_log("[CRYPTO] DTHE AES GCM Decryption failed\r\n");
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    if(*testStatus == SystemP_SUCCESS)
    {
        DebugP_log("[CRYPTO] DTHE AES GCM Decryption and Authentication Success!\r\n");
    }

    return status;
}
