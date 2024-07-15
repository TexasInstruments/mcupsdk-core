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

/* This example demonstrates the DTHE AES 256 ctr Encryption and Decryptions. */

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
#define APP_CRYPTO_AES_CTR_INOUT_LENGTH1                (16U)
#define APP_CRYPTO_AES_CTR_INOUT_LENGTH2                (32U)
#define APP_CRYPTO_AES_CTR_INOUT_LENGTH3                (36U)
/* IV length*/
#define APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES        (4U)
#define APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES           (8U)
#define APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES      (4U)

/* AES CTR KEY length in bytes */
#define APP_CRYPTO_AES_CTR_128_KEY_LENGTH_IN_BYTES      (16U)

/* AES CTR KEY Cache alignment size */
#define APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT              (32U)
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

/* Testing https://www.ietf.org/rfc/rfc3686.txt */

/* Counter Init Value */
static uint8_t gCryptoAesCtrCounterValue[APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x00, 0x00, 0x00, 0x01
};

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCtrPlainText1[APP_CRYPTO_AES_CTR_INOUT_LENGTH1] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x53U, 0x69U, 0x6EU, 0x67U, 0x6CU, 0x65U, 0x20U, 0x62U, 0x6CU, 0x6FU, 0x63U, 0x6BU, 0x20U, 0x6DU, 0x73U, 0x67U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr128Key1[16U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xAEU, 0x68U, 0x52U, 0xF8U, 0x12U, 0x10U, 0x67U, 0xCCU, 0x4BU, 0xF7U, 0xA5U, 0x76U, 0x55U, 0x77U, 0xF3U, 0x9EU
};

static uint8_t gCryptoNonce1[4U] = { 0x00U, 0x00U, 0x00U, 0x30U };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv1[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

static uint8_t gCryptoAesCtr128EncryptedText1[APP_CRYPTO_AES_CTR_INOUT_LENGTH1] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xE4U, 0x09U, 0x5DU, 0x4FU, 0xB7U, 0xA7U, 0xB3U, 0x79U, 0x2DU, 0x61U, 0x75U, 0xA3U, 0x26U, 0x13U, 0x11U, 0xB8U
};

/****************************************************************************************************************************************************************/

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCtrPlainText2[APP_CRYPTO_AES_CTR_INOUT_LENGTH2] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU, 0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr128Key2[16U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x7EU, 0x24U, 0x06U, 0x78U, 0x17U, 0xFAU, 0xE0U, 0xD7U, 0x43U, 0xD6U, 0xCEU, 0x1FU, 0x32U, 0x53U, 0x91U, 0x63U
};

static uint8_t gCryptoNonce2[4U] = { 0x00U, 0x6CU, 0xB6U, 0xDBU };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv2[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xC0U, 0x54U, 0x3BU, 0x59U, 0xDAU, 0x48U, 0xD9U, 0x0BU
};

static uint8_t gCryptoAesCtr128EncryptedText2[APP_CRYPTO_AES_CTR_INOUT_LENGTH2] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x51U, 0x04U, 0xA1U, 0x06U, 0x16U, 0x8AU, 0x72U, 0xD9U, 0x79U, 0x0DU, 0x41U, 0xEEU, 0x8EU, 0xDAU, 0xD3U, 0x88U, 0xEBU, 0x2EU, 0x1EU, 0xFCU, 0x46U, 0xDAU, 0x57U, 0xC8U, 0xFCU, 0xE6U, 0x30U, 0xDFU, 0x91U, 0x41U, 0xBEU, 0x28U
};

/****************************************************************************************************************************************************************/

/* Input buffer for encryption or decryption */
static uint8_t gCryptoAesCtrPlainText3[APP_CRYPTO_AES_CTR_INOUT_LENGTH3] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU, 0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U, 0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU, 0x20U, 0x21U, 0x22U, 0x23U
};

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr128Key3[16U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x76U, 0x91U, 0xBEU, 0x03U, 0x5EU, 0x50U, 0x20U, 0xA8U, 0xACU, 0x6EU, 0x61U, 0x85U, 0x29U, 0xF9U, 0xA0U, 0xDCU
};

static uint8_t gCryptoNonce3[4U] = { 0x00U, 0xE0U, 0x01U, 0x7BU };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv3[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x27U, 0x77U, 0x7FU, 0x3FU, 0x4AU, 0x17U, 0x86U, 0xF0U
};

static uint8_t gCryptoAesCtr128EncryptedText3[APP_CRYPTO_AES_CTR_INOUT_LENGTH3] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xC1U, 0xCFU, 0x48U, 0xA8U, 0x9FU, 0x2FU, 0xFDU, 0xD9U, 0xCFU, 0x46U, 0x52U, 0xE9U, 0xEFU, 0xDBU, 0x72U, 0xD7U, 0x45U, 0x40U, 0xA4U, 0x2BU, 0xDEU, 0x6DU, 0x78U, 0x36U, 0xD5U, 0x9AU, 0x5CU, 0xEAU, 0xAEU, 0xF3U, 0x10U, 0x53U, 0x25U, 0xB2U, 0x07U, 0x2FU
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr192Key1[24U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x16U, 0xAFU, 0x5BU, 0x14U, 0x5FU, 0xC9U, 0xF5U, 0x79U, 0xC1U, 0x75U, 0xF9U, 0x3EU, 0x3BU, 0xFBU, 0x0EU, 0xEDU, 0x86U, 0x3DU, 0x06U, 0xCCU, 0xFDU, 0xB7U, 0x85U, 0x15U
};

static uint8_t gCryptoNonce4[4U] = { 0x00U, 0x00U, 0x00U, 0x48U };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv4[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x36U, 0x73U, 0x3CU, 0x14U, 0x7DU, 0x6DU, 0x93U, 0xCBU
};

static uint8_t gCryptoAesCtr192EncryptedText1[APP_CRYPTO_AES_CTR_INOUT_LENGTH1] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x4BU, 0x55U, 0x38U, 0x4FU, 0xE2U, 0x59U, 0xC9U, 0xC8U, 0x4EU, 0x79U, 0x35U, 0xA0U, 0x03U, 0xCBU, 0xE9U, 0x28U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr192Key2[24U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x7CU, 0x5CU, 0xB2U, 0x40U, 0x1BU, 0x3DU, 0xC3U, 0x3CU, 0x19U, 0xE7U, 0x34U, 0x08U, 0x19U, 0xE0U, 0xF6U, 0x9CU, 0x67U, 0x8CU, 0x3DU, 0xB8U, 0xE6U, 0xF6U, 0xA9U, 0x1AU
};

static uint8_t gCryptoNonce5[4U] = { 0x00U, 0x96U, 0xB0U, 0x3BU };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv5[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x02U, 0x0CU, 0x6EU, 0xADU, 0xC2U, 0xCBU, 0x50U, 0x0DU
};

static uint8_t gCryptoAesCtr192EncryptedText2[APP_CRYPTO_AES_CTR_INOUT_LENGTH2] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x45U, 0x32U, 0x43U, 0xFCU, 0x60U, 0x9BU, 0x23U, 0x32U, 0x7EU, 0xDFU, 0xAAU, 0xFAU, 0x71U, 0x31U, 0xCDU, 0x9FU, 0x84U, 0x90U, 0x70U, 0x1CU, 0x5AU, 0xD4U, 0xA7U, 0x9CU, 0xFCU, 0x1FU, 0xE0U, 0xFFU, 0x42U, 0xF4U, 0xFBU, 0x00U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr192Key3[24U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x02U, 0xBFU, 0x39U, 0x1EU, 0xE8U, 0xECU, 0xB1U, 0x59U, 0xB9U, 0x59U, 0x61U, 0x7BU, 0x09U, 0x65U, 0x27U, 0x9BU, 0xF5U, 0x9BU, 0x60U, 0xA7U, 0x86U, 0xD3U, 0xE0U, 0xFEU
};

static uint8_t gCryptoNonce6[4U] = { 0x00U, 0x07U, 0xBDU, 0xFDU };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv6[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x5CU, 0xBDU, 0x60U, 0x27U, 0x8DU, 0xCCU, 0x09U, 0x12U
};

static uint8_t gCryptoAesCtr192EncryptedText3[APP_CRYPTO_AES_CTR_INOUT_LENGTH3] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x96U, 0x89U, 0x3FU, 0xC5U, 0x5EU, 0x5CU, 0x72U, 0x2FU, 0x54U, 0x0BU, 0x7DU, 0xD1U, 0xDDU, 0xF7U, 0xE7U, 0x58U, 0xD2U, 0x88U, 0xBCU, 0x95U, 0xC6U, 0x91U, 0x65U, 0x88U, 0x45U, 0x36U, 0xC8U, 0x11U, 0x66U, 0x2FU, 0x21U, 0x88U, 0xABU, 0xEEU, 0x09U, 0x35U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr256Key1[32U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x77U, 0x6BU, 0xEFU, 0xF2U, 0x85U, 0x1DU, 0xB0U, 0x6FU, 0x4CU, 0x8AU, 0x05U, 0x42U, 0xC8U, 0x69U, 0x6FU, 0x6CU, 0x6AU, 0x81U, 0xAFU, 0x1EU, 0xECU, 0x96U, 0xB4U, 0xD3U, 0x7FU, 0xC1U, 0xD6U, 0x89U, 0xE6U, 0xC1U, 0xC1U, 0x04U
};

static uint8_t gCryptoNonce7[4U] = { 0x00U, 0x00U, 0x00U, 0x60U };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv7[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xDBU, 0x56U, 0x72U, 0xC9U, 0x7AU, 0xA8U, 0xF0U, 0xB2U
};

static uint8_t gCryptoAesCtr256EncryptedText1[APP_CRYPTO_AES_CTR_INOUT_LENGTH1] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x14U, 0x5AU, 0xD0U, 0x1DU, 0xBFU, 0x82U, 0x4EU, 0xC7U, 0x56U, 0x08U, 0x63U, 0xDCU, 0x71U, 0xE3U, 0xE0U, 0xC0U
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr256Key2[32U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xF6U, 0xD6U, 0x6DU, 0x6BU, 0xD5U, 0x2DU, 0x59U, 0xBBU, 0x07U, 0x96U, 0x36U, 0x58U, 0x79U, 0xEFU, 0xF8U, 0x86U, 0xC6U, 0x6DU, 0xD5U, 0x1AU, 0x5BU, 0x6AU, 0x99U, 0x74U, 0x4BU, 0x50U, 0x59U, 0x0CU, 0x87U, 0xA2U, 0x38U, 0x84U
};

static uint8_t gCryptoNonce8[4U] = { 0x00U, 0xFAU, 0xACU, 0x24U };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv8[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xC1U, 0x58U, 0x5EU, 0xF1U, 0x5AU, 0x43U, 0xD8U, 0x75U
};

static uint8_t gCryptoAesCtr256EncryptedText2[APP_CRYPTO_AES_CTR_INOUT_LENGTH2] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xF0U, 0x5EU, 0x23U, 0x1BU, 0x38U, 0x94U, 0x61U, 0x2CU, 0x49U, 0xEEU, 0x00U, 0x0BU, 0x80U, 0x4EU, 0xB2U, 0xA9U, 0xB8U, 0x30U, 0x6BU, 0x50U, 0x8FU, 0x83U, 0x9DU, 0x6AU, 0x55U, 0x30U, 0x83U, 0x1DU, 0x93U, 0x44U, 0xAFU, 0x1CU
};

/****************************************************************************************************************************************************************/

/* The AES algorithm encrypts and decrypts data in blocks of 128 bits. It can do this using 128-bit or 256-bit keys */
static uint8_t gCryptoAesCtr256Key3[32U] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xFFU, 0x7AU, 0x61U, 0x7CU, 0xE6U, 0x91U, 0x48U, 0xE4U, 0xF1U, 0x72U, 0x6EU, 0x2FU, 0x43U, 0x58U, 0x1DU, 0xE2U, 0xAAU, 0x62U, 0xD9U, 0xF8U, 0x05U, 0x53U, 0x2EU, 0xDFU, 0xF1U, 0xEEU, 0xD6U, 0x87U, 0xFBU, 0x54U, 0x15U, 0x3DU
};

static uint8_t gCryptoNonce9[4U] = { 0x00U, 0x1CU, 0xC5U, 0xB7U };

/* Initialization vector (IV) is an arbitrary number that can be used along with a secret key for data encryption/decryption. */
static uint8_t gCryptoAesCtrIv9[APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0x51U, 0xA5U, 0x1DU, 0x70U, 0xA1U, 0xC1U, 0x11U, 0x48U
};

static uint8_t gCryptoAesCtr256EncryptedText3[APP_CRYPTO_AES_CTR_INOUT_LENGTH3] __attribute__ ((aligned (APP_CRYPTO_AES_CTR_CACHE_ALIGNMENT))) =
{
    0xEBU, 0x6CU, 0x52U, 0x82U, 0x1DU, 0x0BU, 0xBBU, 0xF7U, 0xCEU, 0x75U, 0x94U, 0x46U, 0x2AU, 0xCAU, 0x4FU, 0xAAU, 0xB4U, 0x07U, 0xDFU, 0x86U, 0x65U, 0x69U, 0xFDU, 0x07U, 0xF4U, 0x8CU, 0xC0U, 0xB5U, 0x83U, 0xD6U, 0x07U, 0x1FU, 0x1EU, 0xC0U, 0xE6U, 0xB8U
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void crypto_aes_ctr_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DTHE_AES_Return_t   status;
    DTHE_Handle         aesHandle;
    DTHE_AES_Params     aesParams;
    uint32_t            aesResult1[APP_CRYPTO_AES_CTR_INOUT_LENGTH1/4U];
    uint32_t            aesResult2[APP_CRYPTO_AES_CTR_INOUT_LENGTH2/4U];
    uint32_t            aesResult3[APP_CRYPTO_AES_CTR_INOUT_LENGTH3/4U];
    uint32_t            comparetest;

    /* opens DTHE driver */
    aesHandle = DTHE_open(0);
    DebugP_assert(aesHandle != NULL);

    DebugP_log("[CRYPTO] DTHE AES CTR-128 example started ...\r\n");

    /* Initialize the AES Parameters: */
    (void)memset ((void *)&aesParams, 0, sizeof(DTHE_AES_Params));

    /* Initialize the results: We set the result to a non-zero value. */
    (void)memset ((void *)&aesResult1[0], 0xFF, sizeof(aesResult1));
    (void)memset ((void *)&aesResult2[0], 0xFF, sizeof(aesResult2));
    (void)memset ((void *)&aesResult3[0], 0xFF, sizeof(aesResult3));

    uint8_t ivRegisterValue[16U] = {0};

    memcpy(&ivRegisterValue[0U], &gCryptoNonce1[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv1[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr128Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText1[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr128EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH1);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce2[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv2[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr128Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText2[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr128EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH2);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce3[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv3[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr128Key3[0];
    aesParams.keyLen            = DTHE_AES_KEY_128_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult3[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH3;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText3[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr128EncryptedText3[0], (void *)&aesResult3[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH3);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

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

    DebugP_log("[CRYPTO] DTHE AES CTR-128 example completed!!\r\n");

    DebugP_log("[CRYPTO] DTHE AES CTR-192 example started!!\r\n");

    memcpy(&ivRegisterValue[0U], &gCryptoNonce4[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv4[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr192Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText1[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr192EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH1);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce5[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv5[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr192Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText2[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr192EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH2);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce6[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv6[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr192Key3[0];
    aesParams.keyLen            = DTHE_AES_KEY_192_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult3[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH3;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText3[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr192EncryptedText3[0], (void *)&aesResult3[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH3);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    DebugP_log("[CRYPTO] DTHE AES CTR-192 example completed!!\r\n");

    DebugP_log("[CRYPTO] DTHE AES CTR-256 example started!!\r\n");

    memcpy(&ivRegisterValue[0U], &gCryptoNonce7[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv7[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr256Key1[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult1[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH1;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText1[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr256EncryptedText1[0], (void *)&aesResult1[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH1);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce8[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv8[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr256Key2[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult2[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH2;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText2[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr256EncryptedText2[0], (void *)&aesResult2[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH2);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    memcpy(&ivRegisterValue[0U], &gCryptoNonce9[0U], APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES], &gCryptoAesCtrIv9[0U], APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES);
    memcpy(&ivRegisterValue[APP_CRYPTO_AES_CTR_NONCE_LENGTH_IN_BYTES+APP_CRYPTO_AES_CTR_IV_LENGTH_IN_BYTES], &gCryptoAesCtrCounterValue[0U], APP_CRYPTO_AES_CTR_COUNTER_LENGTH_IN_BYTES);

    /* Initialize the decryption parameters */
    aesParams.algoType          = DTHE_AES_CTR_MODE;
    aesParams.opType            = DTHE_AES_ENCRYPT;
    aesParams.ptrKey            = (uint32_t*)&gCryptoAesCtr256Key3[0];
    aesParams.keyLen            = DTHE_AES_KEY_256_SIZE;
    aesParams.ptrIV             = (uint32_t*)&ivRegisterValue[0];
    aesParams.ptrEncryptedData  = (uint32_t*)&aesResult3[0];
    aesParams.dataLenBytes      = APP_CRYPTO_AES_CTR_INOUT_LENGTH3;
    aesParams.ptrPlainTextData  = (uint32_t*)&gCryptoAesCtrPlainText3[0];
    aesParams.useKEKMode        = FALSE;
    aesParams.counterWidth      = DTHE_AES_CTR_WIDTH_32;

    /* Opens aes driver */
    status = DTHE_AES_open(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    /* Decryption */
    status = DTHE_AES_execute(aesHandle, &aesParams);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    comparetest = memcmp ((void *)&gCryptoAesCtr256EncryptedText3[0], (void *)&aesResult3[0], APP_CRYPTO_AES_CTR_INOUT_LENGTH3);

    /* Comaring Aes operation result with expected result */
    if (comparetest == 0)
    {
        status = DTHE_AES_RETURN_SUCCESS;
    }
    else
    {
        status = DTHE_AES_RETURN_FAILURE;
    }

    /* Closing AES Driver */
    status = DTHE_AES_close(aesHandle);
    DebugP_assert(DTHE_AES_RETURN_SUCCESS == status);

    DebugP_log("[CRYPTO] DTHE AES CTR-256 example completed!!\r\n");

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}
