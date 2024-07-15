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

/* This example demonstrates the DTHE AES CFB Encryption and Decryptions. */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
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
#define APP_CRYPTO_AES_CFB_INOUT_LENGTH_1                (16U)
#define APP_CRYPTO_AES_CFB_INOUT_LENGTH_2                (80U)

/* IV length*/
#define APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES           (16U)

/* AES CFB KEY Cache alignment size */
#define APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT              (32U)
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

/* Testing https://csrc.nist.gov/CSRC/media/Projects/Cryptographic-Algorithm-Validation-Program/documents/aes/aesmmt.zip */

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCfb128PlainText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x4BU, 0x5AU, 0x87U, 0x22U, 0x60U, 0x29U, 0x33U, 0x12U, 0xEEU, 0xA1U, 0xA5U, 0x70U, 0xFDU, 0x39U, 0xC7U, 0x88U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb128Key1[16U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x08U, 0x5bU, 0x8aU, 0xf6U, 0x78U, 0x8fU, 0xa6U, 0xbcU, 0x1aU, 0x0bU, 0x47U, 0xdcU, 0xf5U, 0x0fU, 0xbdU, 0x35U
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb128Iv1[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x58U, 0xcbU, 0x2bU, 0x12U, 0xbbU, 0x52U, 0xc6U, 0xf1U, 0x4bU, 0x56U, 0xdaU, 0x92U, 0x10U, 0x52U, 0x48U, 0x64U
};

static uint8_t gCryptoAesCfb128EncryptedText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xE9U, 0x2CU, 0x80U, 0xE0U, 0xCFU, 0xB6U, 0xD8U, 0xB1U, 0xC2U, 0x7FU, 0xD5U, 0x8BU, 0xC3U, 0x70U, 0x8BU, 0x16U
};

/****************************************************************************************************************************************************************/

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCfb128PlainText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x9EU, 0x69U, 0x42U, 0x36U, 0x53U, 0xC2U, 0x0CU, 0x98U, 0x27U, 0x94U, 0xEDU, 0x35U, 0xD6U, 0x3CU, 0x1AU, 0x78U, 0xE8U, 0xACU, 0x14U, 0xF3U, 0x7EU, 0x18U, 0x88U, 0xAEU, 0x4BU, 0xF2U, 0x73U, 0xBFU, 0xE1U, 0x19U, 0x89U, 0x1BU, 0x2EU, 0x4EU, 0xD8U, 0xACU, 0x46U, 0xE7U, 0xA9U, 0xA4U, 0x63U, 0xC7U, 0xA7U, 0x10U, 0x29U, 0x8DU, 0x43U, 0xB0U, 0x2FU, 0x0CU, 0x56U, 0x06U, 0xBCU, 0xFCU, 0x08U, 0xADU, 0xCEU, 0xEEU, 0xF2U, 0xECU, 0x61U, 0x86U, 0x7FU, 0x8BU, 0xEDU, 0xE4U, 0x98U, 0xE5U, 0x31U, 0x63U, 0x80U, 0x3FU, 0x2FU, 0x86U, 0xFCU, 0x58U, 0x78U, 0x2FU, 0xB8U, 0x41U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb128Key2[16U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x59U, 0x47U, 0xbbU, 0xd7U, 0x8bU, 0x06U, 0xbbU, 0x5eU, 0xa2U, 0xfcU, 0x67U, 0xedU, 0x7bU, 0x24U, 0x21U, 0x6eU
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb128Iv2[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x8eU, 0x47U, 0x22U, 0xadU, 0x22U, 0x30U, 0xb1U, 0x5fU, 0x2eU, 0xeaU, 0x30U, 0x21U, 0x73U, 0xbcU, 0x17U, 0x95U
};

static uint8_t gCryptoAesCfb128EncryptedText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xB2U, 0x3FU, 0x0FU, 0xDFU, 0xC1U, 0x51U, 0x9CU, 0x40U, 0x8EU, 0xE7U, 0xA8U, 0xBAU, 0x46U, 0xEAU, 0x79U, 0xF2U, 0xCBU, 0xEAU, 0x00U, 0x32U, 0x68U, 0x5AU, 0xF8U, 0x2FU, 0x76U, 0xA7U, 0xE2U, 0xB3U, 0x77U, 0x74U, 0x1AU, 0xAAU, 0x61U, 0x8EU, 0xF3U, 0x95U, 0x3EU, 0xDBU, 0xE3U, 0x9EU, 0x8DU, 0xF1U, 0xDDU, 0x28U, 0x3BU, 0x2EU, 0x54U, 0xA0U, 0xF1U, 0x32U, 0x7CU, 0xE3U, 0x32U, 0x18U, 0x8FU, 0x65U, 0x72U, 0x57U, 0x4CU, 0xE5U, 0x94U, 0x28U, 0x63U, 0x6FU, 0x3BU, 0x6EU, 0x37U, 0x05U, 0x4AU, 0x47U, 0x05U, 0xB0U, 0x2BU, 0xEDU, 0xF3U, 0x77U, 0xE4U, 0x65U, 0xE5U, 0xF6U
};

/****************************************************************************************************************************************************************/

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCfb192PlainText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xB4U, 0xE4U, 0x99U, 0xDEU, 0x51U, 0xE6U, 0x46U, 0xFAU, 0xD8U, 0x00U, 0x30U, 0xDAU, 0x9DU, 0xC5U, 0xE7U, 0xE2U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb192Key1[24U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x1bU, 0xbbU, 0x30U, 0x01U, 0x6dU, 0x3aU, 0x90U, 0x88U, 0x27U, 0x69U, 0x33U, 0x52U, 0xecU, 0xe9U, 0x83U, 0x34U, 0x15U, 0x43U, 0x36U, 0x18U, 0xb1U, 0xd9U, 0x75U, 0x95U
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb192Iv1[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xb2U, 0xb4U, 0x8eU, 0x8dU, 0x60U, 0x24U, 0x0bU, 0xf2U, 0xd9U, 0xfaU, 0x05U, 0xccU, 0x2fU, 0x90U, 0xc1U, 0x61U
};

static uint8_t gCryptoAesCfb192EncryptedText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x8BU, 0x7BU, 0xA9U, 0x89U, 0x82U, 0x06U, 0x3AU, 0x55U, 0xFCU, 0xA3U, 0x49U, 0x22U, 0x69U, 0xBBU, 0xE4U, 0x37U
};

/****************************************************************************************************************************************************************/

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCfb192PlainText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xB5U, 0xECU, 0x2EU, 0xA7U, 0xAFU, 0xAEU, 0x21U, 0x67U, 0xE7U, 0xDAU, 0x5BU, 0xC2U, 0xFDU, 0x68U, 0x81U, 0x1AU, 0xD8U, 0x6EU, 0xE5U, 0xC6U, 0x83U, 0x9FU, 0xFEU, 0xB7U, 0x3BU, 0x12U, 0x16U, 0x5CU, 0xC6U, 0x46U, 0x43U, 0xC4U, 0x06U, 0x62U, 0x98U, 0x03U, 0xCDU, 0xC1U, 0x9CU, 0xD6U, 0xF3U, 0xADU, 0xFBU, 0x8AU, 0xA6U, 0x6BU, 0x7CU, 0x19U, 0x02U, 0x79U, 0x33U, 0x97U, 0xE1U, 0x13U, 0xF8U, 0xCCU, 0xF5U, 0xFBU, 0x18U, 0x23U, 0x14U, 0x7AU, 0x4AU, 0xC3U, 0xD2U, 0xA2U, 0xE4U, 0xFBU, 0x55U, 0xD7U, 0x4EU, 0xE3U, 0x65U, 0x8EU, 0xB7U, 0x40U, 0xC3U, 0x53U, 0x08U, 0xA9U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb192Key2[24U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x91U, 0xb7U, 0xe6U, 0xa2U, 0x05U, 0xc6U, 0xb3U, 0x90U, 0x7bU, 0xe7U, 0x09U, 0xa0U, 0x52U, 0x8aU, 0xecU, 0xd9U, 0x49U, 0xffU, 0xb7U, 0x33U, 0x45U, 0x2fU, 0x06U, 0xf1U
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb192Iv2[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x38U, 0xcdU, 0x83U, 0x2dU, 0x3bU, 0xa1U, 0xb0U, 0xeeU, 0x67U, 0x0eU, 0xd3U, 0x85U, 0xd9U, 0x4eU, 0x8eU, 0x25U
};

static uint8_t gCryptoAesCfb192EncryptedText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xCDU, 0xF4U, 0x2CU, 0x51U, 0xD7U, 0xCFU, 0x77U, 0xFBU, 0x5EU, 0x8EU, 0x9EU, 0x98U, 0xABU, 0xB8U, 0xA5U, 0x4EU, 0xADU, 0x22U, 0xFCU, 0xCCU, 0xE9U, 0xECU, 0x9DU, 0x40U, 0x49U, 0x55U, 0x0EU, 0x4FU, 0xBAU, 0x48U, 0xAFU, 0x42U, 0xB0U, 0x32U, 0x74U, 0xF3U, 0x18U, 0xE5U, 0x8AU, 0x9DU, 0xF5U, 0x41U, 0xA7U, 0xE8U, 0xD6U, 0x0AU, 0x78U, 0x12U, 0x39U, 0x54U, 0x2FU, 0xD4U, 0xCBU, 0x8DU, 0xBDU, 0xF7U, 0xACU, 0xF9U, 0xE7U, 0x40U, 0x13U, 0x49U, 0xE5U, 0xF5U, 0x11U, 0x8CU, 0xDBU, 0x5EU, 0xE6U, 0x4BU, 0x3DU, 0x08U, 0x3CU, 0xF6U, 0x5EU, 0x67U, 0x08U, 0x0FU, 0x7BU, 0x83U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb256Key1[32U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xe1U, 0xc6U, 0xe6U, 0x88U, 0x4eU, 0xeeU, 0x69U, 0x55U, 0x2dU, 0xbfU, 0xeeU, 0x21U, 0xf2U, 0x2cU, 0xa9U, 0x26U, 0x85U, 0xd5U, 0xd0U, 0x8eU, 0xf0U, 0xe3U, 0xf3U, 0x7eU, 0x5bU, 0x33U, 0x8cU, 0x53U, 0x3bU, 0xb8U, 0xd7U, 0x2cU
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb256PlainText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xB7U, 0x26U, 0x06U, 0xC9U, 0x8DU, 0x8EU, 0x4FU, 0xABU, 0xF0U, 0x88U, 0x39U, 0xABU, 0xF7U, 0xA0U, 0xACU, 0x61U
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb256Iv1[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xceU, 0xa9U, 0xf2U, 0x3aU, 0xe8U, 0x7aU, 0x63U, 0x7aU, 0xb0U, 0xcdU, 0xa6U, 0x38U, 0x1eU, 0xccU, 0x12U, 0x02U
};

static uint8_t gCryptoAesCfb256EncryptedText1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x29U, 0x81U, 0x76U, 0x1DU, 0x97U, 0x9BU, 0xB1U, 0x76U, 0x5AU, 0x28U, 0xB2U, 0xDDU, 0x19U, 0x12U, 0x5BU, 0x54U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb256Key2[32U] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0x29U, 0x84U, 0x9bU, 0x91U, 0xedU, 0x55U, 0x18U, 0x89U, 0xf7U, 0x9eU, 0x46U, 0x25U, 0xf6U, 0x63U, 0xe8U, 0xa6U, 0x78U, 0x11U, 0x8cU, 0xf8U, 0x08U, 0x0eU, 0xc9U, 0xe4U, 0x9aU, 0x93U, 0xc6U, 0x87U, 0x2aU, 0xbeU, 0xfbU, 0x03U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCfb256PlainText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xBCU, 0xC7U, 0xD5U, 0x08U, 0x86U, 0x39U, 0x4BU, 0x83U, 0xC2U, 0xC0U, 0x48U, 0x18U, 0x9DU, 0xA1U, 0x8BU, 0xDEU, 0xE2U, 0x4EU, 0x4FU, 0x71U, 0xCCU, 0xE8U, 0xC2U, 0x98U, 0x18U, 0x02U, 0xB8U, 0x25U, 0x5DU, 0xA8U, 0xA1U, 0x1DU, 0xA0U, 0x9FU, 0xB8U, 0x19U, 0x23U, 0xEFU, 0x7EU, 0xECU, 0x7FU, 0x9FU, 0x67U, 0x86U, 0xF2U, 0xC0U, 0xD3U, 0x19U, 0xFAU, 0xADU, 0x3FU, 0x7AU, 0x23U, 0x90U, 0xBDU, 0xC9U, 0x1DU, 0x28U, 0xA1U, 0x54U, 0x11U, 0xCAU, 0x7EU, 0xB9U, 0x35U, 0xA6U, 0x51U, 0x4BU, 0x9FU, 0x4CU, 0xFBU, 0xB7U, 0x57U, 0x6FU, 0xF5U, 0xE8U, 0xAFU, 0xE8U, 0x3FU, 0x27U
};

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCfb256Iv2[APP_CRYPTO_AES_CFB_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xadU, 0xdeU, 0x8eU, 0xa7U, 0x0eU, 0x35U, 0x60U, 0xedU, 0x4bU, 0x8fU, 0x5fU, 0x0cU, 0xdbU, 0x5aU, 0xceU, 0x8aU
};

static uint8_t gCryptoAesCfb256EncryptedText2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2] __attribute__ ((aligned (APP_CRYPTO_AES_CFB_CACHE_ALIGNMENT))) =
{
    0xF9U, 0x75U, 0xEDU, 0xF2U, 0xD5U, 0xCAU, 0x72U, 0x82U, 0x59U, 0x98U, 0x67U, 0xC4U, 0xCDU, 0xDBU, 0x7BU, 0x99U, 0xDEU, 0x01U, 0xE3U, 0x57U, 0xEAU, 0x93U, 0xD4U, 0x13U, 0x2BU, 0x6AU, 0x17U, 0x18U, 0xA4U, 0xCBU, 0x3DU, 0x74U, 0x3AU, 0x56U, 0x03U, 0xD6U, 0xAAU, 0x55U, 0x61U, 0x80U, 0xD8U, 0x47U, 0x71U, 0xEDU, 0x5AU, 0x9AU, 0x0CU, 0xAAU, 0x12U, 0xB9U, 0x25U, 0xC9U, 0xD3U, 0xFBU, 0xFEU, 0x4CU, 0x35U, 0x10U, 0x88U, 0x39U, 0x66U, 0x97U, 0xAFU, 0xBEU, 0xF2U, 0x1FU, 0xE0U, 0x77U, 0x58U, 0xC8U, 0xCCU, 0x68U, 0x92U, 0x9BU, 0x7AU, 0xA2U, 0x2AU, 0x6EU, 0x58U, 0x37U
};

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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void crypto_aes_cfb_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_AES_Return_t   status;
    int32_t             testStatus = SystemP_SUCCESS;
    DTHE_Handle         aesHandle;
    DTHE_AES_Params     aesParams;
    uint32_t            aesResult1[APP_CRYPTO_AES_CFB_INOUT_LENGTH_1/4U];
    uint32_t            aesResult2[APP_CRYPTO_AES_CFB_INOUT_LENGTH_2/4U];

    /* opens DTHe driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] DTHE AES CFB-128 example started ...\r\n");

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResult1[0], 0xFF, sizeof(aesResult1));
    (void)memset ((void *)&aesResult2[0], 0xFF, sizeof(aesResult2));

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb128Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb128Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb128PlainText1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb128EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb128Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb128Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb128EncryptedText1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb128PlainText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb128Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb128Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb128PlainText2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb128EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_2) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb128Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb128Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb128EncryptedText2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb128PlainText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    DebugP_log("[CRYPTO] DTHE AES CFB-128 example completed!!\r\n");

    DebugP_log("[CRYPTO] DTHE AES CFB-192 example started!!\r\n");

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb192Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb192Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb192PlainText1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb192EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb192Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb192Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb192EncryptedText1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb192PlainText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb192Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb192Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb192PlainText2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb192EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_2) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb192Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb192Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb192EncryptedText2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb192PlainText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_2) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    DebugP_log("[CRYPTO] DTHE AES CFB-192 example completed!!\r\n");

    DebugP_log("[CRYPTO] DTHE AES CFB-256 example started!!\r\n");

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb256Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb256Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb256PlainText1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb256EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb256Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb256Iv1[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb256EncryptedText1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_1;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult1[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb256PlainText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_1) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Initialize the encryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb256Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb256Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCfb256PlainText2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb256EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_2) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CFB_MODE;
    aesParams.opType            = DTHE_AES_DECRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCfb256Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&gCryptoAesCfb256Iv2[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&gCryptoAesCfb256EncryptedText2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CFB_INOUT_LENGTH_2;
    aesParams.ptrPlainTextData  = (uint32_t*)&aesResult2[0];
    aesParams.useKEKMode        = FALSE;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Comaring Aes operation result with expected result */
    if (memcmp ((void *)&gCryptoAesCfb256PlainText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CFB_INOUT_LENGTH_2) == 0)
    {
        testStatus |= SystemP_SUCCESS;
    }
    else
    {
        testStatus |= SystemP_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    DebugP_log("[CRYPTO] DTHE AES CFB-256 example completed!!\r\n");

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
