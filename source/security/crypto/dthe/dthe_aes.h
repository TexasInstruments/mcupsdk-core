/*
 *  Copyright (C) 2022-24 Texas Instruments Incorporated
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
 *  \defgroup SECURITY_DTHE_AES_MODULE APIs for DTHE AES
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the DTHE AES.
 *
 *  @{
 */

/**
 *  \file dthe_aes.h
 *
 *  \brief This file contains the prototype of DTHE AES driver APIs
 */

#ifndef DTHE_AES_H_
#define DTHE_AES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <security/crypto/dthe/dthe.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr.h>
#include <security/crypto/dthe/hw_include/cslr_aes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Flag for AES ECB Mode */
#define DTHE_AES_ECB_MODE                                   (0x00000000U)
/** \brief Flag for AES CBC Mode */
#define DTHE_AES_CBC_MODE                                   (0x00000001U)
/** \brief Flag for AES CTR Mode */
#define DTHE_AES_CTR_MODE                                   (0x00000002U)
/** \brief Flag for AES ICM Mode */
#define DTHE_AES_ICM_MODE                                   (0x00000004U)
/** \brief Flag for AES CFB Mode */
#define DTHE_AES_CFB_MODE                                   (0x00000008U)
/** \brief Flag for AES F8 Mode */
#define DTHE_AES_F8_MODE                                    (0x00000010U)
/** \brief Flag for AES F9 Mode */
#define DTHE_AES_F9_MODE                                    (0x00000020U)
/** \brief Flag for AES XTS Mode */
#define DTHE_AES_XTS_MODE                                   (0x00000040U)
/** \brief Flag for AES CBC-MAC Mode */
#define DTHE_AES_CBC_MAC_MODE                               (0x00000080U)
/** \brief Flag for AES CMAC Mode */
#define DTHE_AES_CMAC_MODE                                  (0x00000100U)

/** \brief Size of AES key is of 128-bit */
#define DTHE_AES_KEY_128_SIZE                               (0x00000001U)
/** \brief Size of AES key is of 192-bit */
#define DTHE_AES_KEY_192_SIZE                               (0x00000002U)
/** \brief Size of AES key is of 256-bit */
#define DTHE_AES_KEY_256_SIZE                               (0x00000003U)

/** \brief AES Encrypt Flag */
#define DTHE_AES_ENCRYPT                                    (0x016FE45DU)
/** \brief AES Decrypt Flag */
#define DTHE_AES_DECRYPT                                    (0xDCBA4213U)

/** \brief AES CTR Counter Width is 16 (ICM) */
#define DTHE_AES_CTR_WIDTH_16                                    (0x00000001U)
/** \brief AES CTR Counter Width is 32 */
#define DTHE_AES_CTR_WIDTH_32                                    (0x00000002U)
/** \brief AES CTR Counter Width is 64 */
#define DTHE_AES_CTR_WIDTH_64                                    (0x00000004U)
/** \brief AES CTR Counter Width is 96 */
#define DTHE_AES_CTR_WIDTH_96                                    (0x00000008U)
/** \brief AES CTR Counter Width is 128 */
#define DTHE_AES_CTR_WIDTH_128                                   (0x00000010U)

/** \brief AES STREAM SUPPORT */
#define DTHE_AES_ONE_SHOT_SUPPORT                           (0x00000000U)
/** \brief AES STREAM SUPPORT : INIT */
#define DTHE_AES_STREAM_INIT                               (0xAA11BB22U)
/** \brief AES STREAM SUPPORT : UPDATE */
#define DTHE_AES_STREAM_UPDATE                              (0x33CC44DDU)
/** \brief AES STREAM SUPPORT : FINISH */
#define DTHE_AES_STREAM_FINISH                              (0xEE55FF66U)



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief
 *  DTHE AES Driver Error code
 *
 * \details
 *  The enumeration describes all the possible return and error codes which
 *  the DTHE AES Driver can return
 */
typedef enum DTHE_AES_Return_e
{
    DTHE_AES_RETURN_SUCCESS                  = 0x62E699D9U, /*!< Success/pass return code */
    DTHE_AES_RETURN_FAILURE                  = 0x904D041AU, /*!< General or unspecified failure/error */
}DTHE_AES_Return_t;

/**
 * \brief AES Driver Parameters
 *  This structure has all the parameters which are need by the AES Driver
 *  to perform the specified operation.
 *
 *|Parameter          | DTHE_AES_ONE_SHOT_SUPPORT | DTHE_AES_STREAM_INIT | DTHE_AES_STREAM_UPDATE | DTHE_AES_STREAM_FINISH |
* |------------------:|:-------------------------:|:--------------------:|:----------------------:|:----------------------:|
* | algoType          |              *            |           *          |            *           |             *          |
* | opType            |              *            |           *          |            *           |             *          |
* | useKEKMode        |              *            |           *          |                        |                        |
* | ptrKey            |              *            |           *          |                        |                        |
* | ptrKey1           |              *            |           *          |                        |                        |
* | ptrKey2           |              *            |           *          |                        |                        |
* | keyLen            |              *            |           *          |                        |                        |
* | ptrIV             |              *            |           *          |                        |                        |
* | dataLenBytes      |              *            |                      |                        |                        |
* | ptrEncryptedData  |              *            |                      |            *           |             *          |
* | ptrPlainTextData  |              *            |                      |            *           |             *          |
* | counterWidth      |              *            |           *          |                        |                        |
* | streamState       |                           |           *          |            *           |             *          |
* | streamSize        |                           |                      |            *           |             *          |
* | ptrTag            |                           |                      |                        |             *          |
 *
 */
typedef struct DTHE_AES_Params_t
{
    /**< Algorithm to be performed by the AES Driver*/
    uint32_t            algoType;

    /** \brief  Operation to be performed by the AES Driver*/
    uint32_t            opType;

    /**
     *< This is a boolean flag which indicates if the KEK mode is to be used or not. If this is set to TRUE then the 'ptrKey' below is not used.
     * The operation mode is ignored and this is only used for encryption.
     */
    Bool                useKEKMode;

    /**
     *< Pointer to the key to be used to perform the decryption. The driver supports AES-CBC with 256bit keys.
     * This is only valid if the KEK mode flag above is set to be FALSE.
     */
    uint32_t*           ptrKey;

    /**
     *< To be used only for CMAC
     */
    uint32_t*           ptrKey1;

    /**
     *< To be used only for CMAC
     */
    uint32_t*           ptrKey2;

    /**
        \brief   Length of the Key
     */
    uint8_t             keyLen;

    /**
     *<   Pointer to the Initialization Vector to be used.
     */
    uint32_t*           ptrIV;

    /**
     *<   Size of the data in bytes. This value cannot be equal to zero.
     *    For MAC algorithms, which support dataLength as zero bytes, the handling is done outside the driver scope.
     *
     */
    uint32_t            dataLenBytes;

    /**
     *<   Pointer to the encrypted data buffer:
     *
     * Decryption Operation Mode:
     * - This is used as an input parameter and is used to point to the location of the encrypted data buffer.
     *
     * Encryption Operation Mode:
     * - This is used as an output parameter and is the location where the encrypted data will be present.
     *
     * Note : Not valid for AES-CMAC mode.
     */
    uint32_t*           ptrEncryptedData;

    /**
     *<   Pointer to the Plain Text data buffer:
     *
     * Decryption Operation Mode:
     * - This is used as an output parameter and is used to point to the location of the plain text data after the encrypted data has been decrypted.
     *
     * Encryption Operation Mode:
     * - This is used as an input parameter and is the location where the plain text data is present which will be encrypted.
     */
    uint32_t*           ptrPlainTextData;

    /**
     *<   Pointer to Tag
     */
    uint32_t*           ptrTag;

    /**
     *<   Width of Counter in bits
     */
    uint32_t            counterWidth;


    /**
     *<   Only valid for Streaming Support; Valid values are - DTHE_AES_STREAM_INIT, DTHE_AES_STREAM_UPDATE, DTHE_AES_STREAM_FINISH
     */
    uint32_t            streamState;

    /**
     *<   Only valid for Streaming Support
     *
     * - This field is not populated in case of streamState==DTHE_AES_ONE_SHOT_SUPPORT and streamState==DTHE_AES_STREAM_INIT and can be set as 0
     *
     * - In case of streamState == DTHE_AES_STREAM_UPDATE, the streamSize must be aligned to 16 Bytes.
     *
     * - In case of streamState == DTHE_AES_STREAM_FINISH, the streamSize does not have to be aligned to 16 Bytes.
     */
    uint32_t            streamSize;
}DTHE_AES_Params;
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                              Function Definitions                          */
/* ========================================================================== */

/**
 * \brief               Function to Open DTHE AES Driver.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \return              #DTHE_AES_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_AES_RETURN_FAILURE if requested operation not completed.
 */
DTHE_AES_Return_t DTHE_AES_open(DTHE_Handle handle);

/**
 * \brief               The function is used to execute the AES Driver with the specified parameters.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \param ptrParams     Pointer to the parameters to be used to execute the driver.
 *
 * \return              #DTHE_AES_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_AES_RETURN_FAILURE if requested operation not completed.
 */
DTHE_AES_Return_t DTHE_AES_execute(DTHE_Handle handle, const DTHE_AES_Params* ptrParams);

/**
 * \brief               Function to close DTHE AES Driver.
 *
 * \param  handle       #DTHE_Handle returned from #DTHE_open().
 *
 * \return              #DTHE_AES_RETURN_SUCCESS if requested operation completed.
 *                      #DTHE_AES_RETURN_FAILURE if requested operation not completed.
 */
DTHE_AES_Return_t DTHE_AES_close(DTHE_Handle handle);

#ifdef __cplusplus
}
#endif

#endif
/** @} */
