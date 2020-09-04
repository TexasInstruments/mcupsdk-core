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
 *  \defgroup SECURITY_CRYPTO_MODULE APIs for CRYPTO
 *  \ingroup SECURITY_MODULE
 *
 *  @{
 *
 */

/**
 *  \file crypto.h
 *
 *  \brief This file contains the prototype of crypto driver APIs
 */

#ifndef CRYPTO_TOP_H_
#define CRYPTO_TOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Key length for HMAC-SHA Process */
#define CRYPTO_HMAC_SHA_MAX_KEYLEN_BYTES      (128U)
/** \brief Key length for HMAC-SHA512 Process */
#define CRYPTO_HMAC_SHA512_KEYLEN_BYTES       (128U)
/** \brief Key length for HMAC-SHA1 Process */
#define CRYPTO_HMAC_SHA1_KEYLEN_BYTES         (64U)
/** \brief Key length for HMAC-SHA256 Process */
#define CRYPTO_HMAC_SHA256_KEYLEN_BYTES       (64U)
/** \brief Inner Padding value for HMAC */
#define CRYPTO_HMAC_SHA_IPAD                  (0x36U)
/** \brief Outer Padding value for HMAC */
#define CRYPTO_HMAC_SHA_OPAD                  (0x5CU)

/** \brief Aes Cmac key length*/
#define CRYPTO_AES_CMAC_KEY_LENGTH            (16U)
/** \brief Aes block length*/
#define CRYPTO_AES_BLOCK_LENGTH               (16U)

/** \brief Handle to the Crypto driver returned by #Crypto_open() */
typedef void *Crypto_Handle;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief CRYPTO driver context */
typedef struct
{
    void               *drvHandle;
    /**< Driver handler */
} Crypto_Context;

/** \brief CRYPTO Parameters for various operations */
typedef struct
{
    /** Authentication mode.*/
    uint32_t                authMode;
    /** Key for Hmac sha algorithm */
    uint8_t                 key[CRYPTO_HMAC_SHA_MAX_KEYLEN_BYTES];
    /** Key size in bytes for Hmac algorithm */
    uint32_t                keySizeInBytes;
    /** iPad for hmac calculation */
    uint8_t                 iPad[CRYPTO_HMAC_SHA_MAX_KEYLEN_BYTES];
    /** oPad for hmac calculation */
    uint8_t                 oPad[CRYPTO_HMAC_SHA_MAX_KEYLEN_BYTES];
    /** Used to calculate key1 and key2 */
    uint8_t                 aesWithKeyAppliedToZeroInput[CRYPTO_AES_BLOCK_LENGTH];
    /** Key1 for Cmac operations*/
    uint8_t                 key1[CRYPTO_AES_CMAC_KEY_LENGTH];
    /** Key2 for Cmac operations*/
    uint8_t                 key2[CRYPTO_AES_CMAC_KEY_LENGTH];
} Crypto_Params;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the Crypto module
 */
void Crypto_init(void);

/**
 *  \brief  This function de-initializes the Crypto module
 */
void Crypto_deinit(void);

/**
 * \brief           This function opens a Crypto module
 *
 * \param ctx       The Crypto Context to initialize. This must not be \c NULL.
 */
Crypto_Handle Crypto_open(Crypto_Context *ctx);

/**
 * \brief          Function to close a Crypto module
 *
 * \param handle   The Crypto context to clear. This may be \c NULL,
 *                 in which case this function does nothing. If it
 *                 is not \c NULL, it must point to an initialized
 *                 Crypto context.
*/
int32_t Crypto_close(Crypto_Handle handle);

/**
 * \brief          This function calculates oPad & iPad for HMAC.
 *
 *
 * \param params   params for ipad and opad calculation
 *
 * \return status  \c 0 on success.
 *                 A negative error code on failure.
 */
int32_t Crypto_hmacSha(Crypto_Params *params);

/**
 * \brief          This function Generate Sub keys for CMAC calculation.
 *
 * \param params   params for key1 and key2 calculation
 *
 * \return status  \c 0 on success.
 *                 A negative error code on failure.
 */
int32_t Crypto_cmacGenSubKeys(Crypto_Params *params);
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* CRYPTO_TOP_H_ */

/** @} */