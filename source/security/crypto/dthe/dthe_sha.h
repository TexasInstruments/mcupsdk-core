/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \defgroup SECURITY_DTHE_SHA_MODULE APIs for DTHE SHA
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the DTHE SHA.
 *
 *  @{
 */

/**
 *  \file dthe_sha.h
 *
 *  \brief This file contains the prototype of DTHE SHA driver APIs
 */

#ifndef DTHE_SHA_H_
#define DTHE_SHA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <security/crypto/dthe/dthe.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr.h>
#include <security/crypto/dthe/hw_include/cslr_sha.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**< SHA Algorithms */
/** \brief  MD5 Flag */
#define DTHE_SHA_ALGO_MD5                   (0x1U)
/** \brief  SHA-384 Flag */
#define DTHE_SHA_ALGO_SHA384                (0x2U)
/** \brief  SHA-1 Flag */
#define DTHE_SHA_ALGO_SHA1                  (0x3U)
/** \brief  SHA-512 Flag */
#define DTHE_SHA_ALGO_SHA512                (0x4U)
/** \brief  SHA-224 Flag */
#define DTHE_SHA_ALGO_SHA224                (0x5U)
/** \brief  SHA-256 Flag */
#define DTHE_SHA_ALGO_SHA256                (0x6U)

/** \brief The Maximum Digest Size is 64bytes for the SHA512.*/
#define DTHE_SHA_MAX_DIGEST_SIZE_BYTES      (64U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 * \brief
 *  DTHE SHA Driver Error code
 *
 * \details
 *  The enumeration describes all the possible return and error codes which
 *  the DTHE SHA Driver can return
 */
typedef enum DTHE_SHA_Return_e
{
    DTHE_SHA_RETURN_SUCCESS                  = 0x67A42DD1U, /*!< Success/pass return code */
    DTHE_SHA_RETURN_FAILURE                  = 0x06C2B483U, /*!< General or unspecified failure/error */
}DTHE_SHA_Return_t;

/** \brief Parameters required for SHA Driver */
typedef struct DTHE_SHA_Params_t
{
    /** \brief Algorithm to be performed by the SHA Driver */
    uint32_t            algoType;
    /** \brief Pointer to the Plain Text data buffer */
    uint32_t            *ptrDataBuffer;
    /** \brief Size of the data in bytes */
    uint32_t            dataLenBytes;
    /** \brief Pointer to the key for hmac processing */
    uint32_t            *ptrKey;
    /** \brief Size of the key in bytes */
    uint32_t            keySize;
    /** \brief output buffer for storing sha degest */
    uint32_t            digest[DTHE_SHA_MAX_DIGEST_SIZE_BYTES/4U];
}DTHE_SHA_Params;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                              Function Declarations                         */
/* ========================================================================== */

/**
 * \brief               Function to Open DTHE SHA Driver.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \return              #DTHE_SHA_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_SHA_RETURN_FAILURE if requested operation not completed.
 */
DTHE_SHA_Return_t DTHE_SHA_open(DTHE_Handle handle);

/**
 * \brief               The function is used to execute the SHA Driver with the specified parameters.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \param ptrShaParams  Pointer to the parameters to be used to execute the driver.
 *
 * \param isLastBlock   Used for singleshot and multishot sha.
 *
 * \return              #DTHE_SHA_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_SHA_RETURN_FAILURE if requested operation not completed.
 */
DTHE_SHA_Return_t DTHE_SHA_compute(DTHE_Handle handle, DTHE_SHA_Params* ptrShaParams, int32_t isLastBlock);

/**
 * \brief               The function is used to execute the HMAC SHA Operations with the specified parameters.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \param ptrShaParams  Pointer to the parameters to be used to execute the driver.
 *
 * \return              #DTHE_SHA_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_SHA_RETURN_FAILURE if requested operation not completed.
 */
DTHE_SHA_Return_t DTHE_HMACSHA_compute(DTHE_Handle handle, DTHE_SHA_Params* ptrShaParams);

/**
 * \brief               Function to close DTHE SHA Driver.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \return              #DTHE_SHA_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_SHA_RETURN_FAILURE if requested operation not completed.
 */
DTHE_SHA_Return_t DTHE_SHA_close(DTHE_Handle handle);

#ifdef __cplusplus
}
#endif

#endif
/** @} */
