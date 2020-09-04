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
 *  \defgroup SECURITY_CRYPTO_UTIL_MODULE APIs for Crypto utilities
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the PKA.
 *
 *  @{
 */

/**
 *  \file crypto_util.h
 *
 *  \brief This file contains the prototype of crypto_util driver APIs
 */

#ifndef PKA_UTIL_H_
#define PKA_UTIL_H_

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

/**
 *  \anchor Crypto_AlgoTypes
 *  \name crypto Algo Types
 *  @{
 */
/** \brief Hash Algo SHA-1 */
#define HASH_ALG_SHA1             (0x0U)
/** \brief Hash Algo SHA-256 */
#define HASH_ALG_SHA2_256         (0x1U)
/** \brief Hash Algo SHA-512 */
#define HASH_ALG_SHA2_512         (0x2U)
/** @} */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                              Function Declarations                          */
/* ========================================================================== */

/**
 *  \brief  Function to convert uint8_t to uint32_t format
 *
 *  \param  source  Uint8_t type buffer for conversion
 *
 *  \param  sourceLengthInBytes  Length of source buffer in bytes
 *
 *  \param  dest    Resultant uint32_t buffer stored in dest
 */
void Crypto_Uint8ToUint32(const uint8_t *source, uint32_t sourceLengthInBytes, uint32_t *dest);

/**
 *  \brief  Function to convert uint32_t to uint8_t format
 *
 *  \param  src  Uint32_t type buffer for conversion
 *
 *  \param  sourceLengthInBytes  Length of source buffer in bytes
 *
 *  \param  dest Resultant uint8_t buffer stored in dest
 */
void Crypto_Uint32ToUint8(const uint32_t *src, uint32_t sourceLengthInBytes, uint8_t *dest);

/**
 *  \brief  Function to convert uint32_t to Bigint format
 *
 *  \param  source                  uint32_t type buffer for conversion
 *
 *  \param  sourceLengthInWords     length of source buffer in words
 *
 *  \param  dest                    Resultant bigint buffer stored in dest
 */
void Crypto_Uint32ToBigInt(uint32_t *source, uint32_t sourceLengthInWords, uint32_t *dest);

/**
 *  \brief  Function to convert Bigint to uint32_t format
 *
 *  \param  source                  Uint32_t type buffer for conversion
 *
 *  \param  sourceLengthInWords     Length of source buffer in words
 *
 *  \param  dest                    Resultant uint32 buffer stored in dest
 */
void Crypto_bigIntToUint32(uint32_t *source, uint32_t sourceLengthInWords, uint32_t *dest);

/**
 *  \brief  Padding function for sign
 *
 *  \param  shaHash             Calculated Hash of the message for padding
 *
 *  \param  keyLengthInBytes    Used while padding to match key and padded mesage size
 *
 *  \param  typeOfAlgo          Used while padding to check sha length, refer \ref Crypto_AlgoTypes
 *
 *  \param  output              Resultant padded buffer stored in dest
 */
void Crypto_PKCSPaddingForSign(const uint8_t *shaHash, uint32_t keyLengthInBytes, uint32_t typeOfAlgo, uint8_t *output);

/**
 *  \brief  Padding function for Message
 *
 *  \param  message             Message for padding
 *
 *  \param  msgLengthInBytes    Used while padding to match key and padded mesage size
 *
 *  \param  keyLengthInBytes    Used while padding to check key length
 *
 *  \param  output              Resultant padded buffer stored in dest
 */
void Crypto_PKCSPaddingForMessage(const uint8_t *message, uint32_t msgLengthInBytes, uint32_t keyLengthInBytes, uint8_t *output);


#ifdef __cplusplus
}
#endif

#endif /* PKA_UTIL_H_ */
/** @} */
