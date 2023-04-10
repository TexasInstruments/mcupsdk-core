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
 *  \file   dthe_sha.c
 *
 *  \brief  This file contains the implementation of Dthe sha driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <security/crypto/dthe/dthe_sha.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief The Maximum Digest Size is 64bytes for the SHA512. */
#define DTHE_SHA_MAX_DIGEST_SIZE_BYTES          (64U)

/** \brief The Maximum HMAC Key Size is 128bytes or 1024bits for the SHA512. */
#define DTHE_SHA_MAX_HMAC_KEY_SIZE_BYTES        (128U)

/** \brief This is the Block Size for the SHA512 in words */
#define DTHE_SHA512_BLOCK_SIZE                  (32U)

/** \brief This is the Block Size for the SHA256 in words */
#define DTHE_SHA256_BLOCK_SIZE                  (16U)

/** \brief This is the Data Shift Size for the SHA512 */
#define DTHE_SHA512_SHIFT_SIZE                  (5U)

/** \brief This is the Data Shift Size for the SHA256 */
#define DTHE_SHA256_SHIFT_SIZE                  (4U)

/** \brief The Maximum HMAC Key Size is 128bytes or 1024bits for the SHA512. */
#define DTHE_HMAC_SHA_MAX_KEY_SIZE_BYTES        (128U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                           Global variables                                */
/* ========================================================================== */
/** \brief Flag to check SHA in Progress */
Bool                    gDTHESHAInProgress;
/** \brief SHA Digest Count */
uint32_t                gDTHESHAdigestCount;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static void DTHE_SHA_setInterruptStatus(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t intStatus);
static void DTHE_SHA_pollContextReady(CSL_EIP57T_SHARegs* ptrSHARegs);
static uint8_t DTHE_SHA_isContextReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs);
static void DTHE_SHA_setUseAlgoConstants(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t useAlgConstants);
static void DTHE_SHA_setCloseHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t closeHashFlag);
static void DTHE_SHA_setHMACKeyProcessing(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacKeyFlag);
static void DTHE_SHA_setHMACOuterHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacOuterHash);
static void DTHE_SHA512_setUseAlgoConstants(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t useAlgConstants);
static void DTHE_SHA512_setCloseHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t closeHashFlag);
static void DTHE_SHA512_setHMACKeyProcessing(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacKeyFlag);
static void DTHE_SHA512_setHMACOuterHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacOuterHash);
static void DTHE_SHA_setHashLength(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t length);
static void DTHE_SHA512_setHashLength(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t length);
static void DTHE_SHA_pollInputReady(CSL_EIP57T_SHARegs* ptrSHARegs);
static void DTHE_SHA_writeDataBlock(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrDataBlock, uint8_t blockSize);
static void DTHE_SHA_pollOutputReady (CSL_EIP57T_SHARegs* ptrSHARegs);
static void DTHE_SHA_getHashDigest(const CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t* ptrDigest);
static void DTHE_SHA512_getHashDigest(const CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t* ptrDigest);
static void DTHE_SHA_setAlgorithm(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t algorithm);
static uint8_t DTHE_SHA_isOutputReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs);
static uint8_t DTHE_SHA_isInputReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs);
static uint32_t DTHE_SHA512_getDigestCount(const CSL_EIP57T_SHARegs* ptrSHARegs);
static uint32_t DTHE_SHA_getDigestCount(const CSL_EIP57T_SHARegs* ptrSHARegs);
static void DTHE_SHA_setHMACOuterKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey);
static void DTHE_SHA_setHMACInnerKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey);
static void DTHE_SHA512_setHMACOuterKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey);
static void DTHE_SHA512_setHMACInnerKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

DTHE_SHA_Return_t DTHE_SHA_open(DTHE_Handle handle)
{
    DTHE_SHA_Return_t status  = DTHE_SHA_RETURN_FAILURE;
    DTHE_Config     *config = NULL;
    DTHE_Attrs      *attrs  = NULL;
    CSL_EIP57T_SHARegs     *ptrShaRegs;

    if(NULL != handle)
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }

    if(status == DTHE_SHA_RETURN_SUCCESS)
    {
        config              = (DTHE_Config *) handle;
        attrs               = config->attrs;
        ptrShaRegs          = (CSL_EIP57T_SHARegs *)attrs->shaBaseAddr;
        gDTHESHAInProgress  = FALSE;

        /* Disable all interrupts */
        DTHE_SHA_setInterruptStatus(ptrShaRegs, 0U);
    }

    return (status);
}

DTHE_SHA_Return_t DTHE_SHA_close(DTHE_Handle handle)
{
    DTHE_SHA_Return_t   status  = DTHE_SHA_RETURN_FAILURE;
    DTHE_Config         *config = NULL;
    DTHE_Attrs          *attrs  = NULL;
    CSL_EIP57T_SHARegs  *ptrShaRegs;

    if(NULL != handle)
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }

    if(status == DTHE_SHA_RETURN_SUCCESS)
    {
        config              = (DTHE_Config *) handle;
        attrs               = config->attrs;
        ptrShaRegs          = (CSL_EIP57T_SHARegs *)attrs->shaBaseAddr;
        if(gDTHESHAInProgress == FALSE)
        {
            /* Disable all interrupts */
            DTHE_SHA_setInterruptStatus(ptrShaRegs, 0U);

            gDTHESHAInProgress = 0;
            gDTHESHAdigestCount = 0;
        }
        else
        {
            status  = DTHE_SHA_RETURN_FAILURE;
        }
    }

    return (status);
}

DTHE_SHA_Return_t DTHE_SHA_compute(DTHE_Handle handle, DTHE_SHA_Params* ptrShaParams, int32_t isLastBlock)
{
    DTHE_SHA_Return_t       status = DTHE_SHA_RETURN_SUCCESS;
    uint32_t                index = 0U;
    uint32_t                dataLenWords;
    uint32_t                dataLenBytes;
    uint8_t                 *ptrByteDataBuffer;
    uint32_t                numBlocks;
    uint8_t                 useAlgoConstants;
    uint8_t                 closeHash;
    uint8_t                 blockSize;
    uint32_t                shiftSize;
    uint32_t                numPartialWords;
    uint32_t                partialWord = 0U;
    uint32_t                numBytes    = 0U;
    DTHE_Config             *config = NULL;
    DTHE_Attrs              *attrs  = NULL;
    CSL_EIP57T_SHARegs      *ptrShaRegs;

    if((NULL == handle) || (NULL == ptrShaParams))
    {
        status = DTHE_SHA_RETURN_FAILURE;
    }

    if(status == DTHE_SHA_RETURN_SUCCESS)
    {
        config              = (DTHE_Config *) handle;
        attrs               = config->attrs;
        ptrShaRegs          = (CSL_EIP57T_SHARegs *)attrs->shaBaseAddr;
        dataLenBytes        = ptrShaParams->dataLenBytes;

        DTHE_SHA_setAlgorithm(ptrShaRegs, ptrShaParams->algoType);

    }
    /* Sanity Checking: Any data buffer except the last block should be aligned as per
     * the SHA Size. For SHA256 this is 64byte while for SHA512 this should be 128byte */
    if((status == DTHE_SHA_RETURN_SUCCESS) && (isLastBlock == FALSE))
    {
        if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
        {
            if ((dataLenBytes % (DTHE_SHA256_BLOCK_SIZE * sizeof(uint32_t))) != 0U)
            {
                /* Error: Ensure that the data length is a word multiple. */
                status = DTHE_SHA_RETURN_FAILURE;
            }
        }
        else
        {
            if ((dataLenBytes % (DTHE_SHA512_BLOCK_SIZE * sizeof(uint32_t))) != 0U)
            {
                /* Error: Ensure that the data length is a word multiple. */
                status = DTHE_SHA_RETURN_FAILURE;
            }
        }
    }
    else
    {
        /* Last Blocks: These need no alignment. */
    }

    /* Perform the SHA Computation */
    if (status == DTHE_SHA_RETURN_SUCCESS)
    {
        /* Ensure that the SHA IP Block is ready to receive data: */
        DTHE_SHA_pollContextReady (ptrShaRegs);

        /********************************************************************
         * Is this the first block which is being passed to the SHA Engine?
         ********************************************************************/
        if (gDTHESHAInProgress == FALSE)
        {
            /* Yes: For the first block we will use the algorithm constants */
            useAlgoConstants = 1U;
        }
        else
        {
            /* No: For all other blocks we will not use the algorithm constants */
            useAlgoConstants = 0U;
        }

        /* Is this the last block? */
        if (isLastBlock == TRUE)
        {
            /* Yes: Close the Hash */
            closeHash = 1U;
        }
        else
        {
            /* No: Dont close the hash there are more data blocks. */
            closeHash = 0U;
        }

        /* Update the Hash Mode: */
        if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
        {
            DTHE_SHA_setUseAlgoConstants(ptrShaRegs, useAlgoConstants);
            DTHE_SHA_setCloseHash(ptrShaRegs, closeHash);

            /* Reset the HMAC Processing: */
            DTHE_SHA_setHMACKeyProcessing(ptrShaRegs, 0U);
            DTHE_SHA_setHMACOuterHash(ptrShaRegs, 0U);
        }
        else
        {
            DTHE_SHA512_setUseAlgoConstants(ptrShaRegs, useAlgoConstants);
            DTHE_SHA512_setCloseHash(ptrShaRegs, closeHash);

            /* Reset the HMAC Processing: */
            DTHE_SHA512_setHMACKeyProcessing(ptrShaRegs, 0U);
            DTHE_SHA512_setHMACOuterHash(ptrShaRegs, 0U);
        }

        /* Write the length of the data: */
        if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
        {
            /* SHA256: Setup the block & shift size: */
            blockSize = DTHE_SHA256_BLOCK_SIZE;
            shiftSize = DTHE_SHA256_SHIFT_SIZE;

            /* Set the data length: */
            DTHE_SHA_setHashLength(ptrShaRegs, dataLenBytes);
        }
        else
        {
            /* SHA512: Setup the block & shift size: */
            blockSize = DTHE_SHA512_BLOCK_SIZE;
            shiftSize = DTHE_SHA512_SHIFT_SIZE;

            /* Set the data length: */
            DTHE_SHA512_setHashLength(ptrShaRegs, dataLenBytes);
        }

        /* Determine the data length in words: */
        dataLenWords = dataLenBytes / 4U;

        /* Compute the number of blocks: */
        numBlocks = dataLenWords / blockSize;

        /* Compute the number of partial words which need to be handled seperately */
        numPartialWords = dataLenWords % blockSize;

        /* Compute the number of full blocks which need to be processed: */
        for (index = 0U; index < numBlocks; index = index + 1U)
        {
            /* Ensure that the SHA IP Block is ready to receive data: */
            DTHE_SHA_pollInputReady(ptrShaRegs);

            /* Write the data block: */
            DTHE_SHA_writeDataBlock(ptrShaRegs,
                                    &ptrShaParams->ptrDataBuffer[index << shiftSize],
                                    blockSize);

            /* Compute the number of bytes which have been processed: */
            numBytes = numBytes + (blockSize * sizeof(uint32_t));
        }

        /* Process any left over data: */
        if (numPartialWords != 0U)
        {
            /* Ensure that the SHA IP Block is ready to receive data: */
            DTHE_SHA_pollInputReady(ptrShaRegs);

            /* Write the data block: */
            DTHE_SHA_writeDataBlock(ptrShaRegs,
                                    &ptrShaParams->ptrDataBuffer[index << shiftSize],
                                    (uint8_t)numPartialWords);

            /* Compute the number of bytes which have been processed: */
            numBytes = numBytes + (numPartialWords * sizeof(uint32_t));
        }

        /* Do we need to account for some additional bytes? */
        if (dataLenBytes != numBytes)
        {
            /* Get the pointer to the data buffer in bytes which will be written: */
            ptrByteDataBuffer = (uint8_t*)ptrShaParams->ptrDataBuffer;

            /* Copy into the partial word: */
            (void)memcpy ((void *)&partialWord, (void *)&ptrByteDataBuffer[numBytes], (dataLenBytes - numBytes));

            /* Ensure that the SHA IP Block is ready to receive data: */
            DTHE_SHA_pollInputReady(ptrShaRegs);

            /* Write the data block: */
            DTHE_SHA_writeDataBlock(ptrShaRegs, &partialWord, 1U);
        }

        /* Poll till the intermediate hash results are available: */
        DTHE_SHA_pollOutputReady (ptrShaRegs);

        /* Get the digest count and value: */
        if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
        {
            DTHE_SHA_getHashDigest(ptrShaRegs, &ptrShaParams->digest[0]);
            gDTHESHAdigestCount = DTHE_SHA_getDigestCount(ptrShaRegs);
        }
        else
        {
            DTHE_SHA512_getHashDigest(ptrShaRegs, &ptrShaParams->digest[0]);
            gDTHESHAdigestCount = DTHE_SHA512_getDigestCount(ptrShaRegs);
        }

        if( isLastBlock == TRUE )
        {
            /* SHA Computation is in progress: */
            gDTHESHAInProgress = FALSE;
        }
        else
        {
            /* SHA Computation is in progress: */
            gDTHESHAInProgress = TRUE;
        }
    }
    return (status);
}

DTHE_SHA_Return_t DTHE_HMACSHA_compute(DTHE_Handle handle, DTHE_SHA_Params* ptrShaParams)
{
    DTHE_SHA_Return_t       status = DTHE_SHA_RETURN_FAILURE;
    uint32_t                index;
    uint32_t                dataLenWords;
    uint32_t                dataLenBytes;
    uint8_t                 *ptrByteDataBuffer;
    uint32_t                numBlocks;
    uint8_t                 blockSize;
    uint32_t                shiftSize;
    uint32_t                numPartialWords;
    uint32_t                partialWord = 0U;
    uint32_t                numBytes    = 0U;
    uint32_t                hmacPaddedKey[DTHE_HMAC_SHA_MAX_KEY_SIZE_BYTES/4U];
    DTHE_Config             *config = NULL;
    DTHE_Attrs              *attrs  = NULL;
    CSL_EIP57T_SHARegs      *ptrShaRegs;

    if((NULL != handle) && (NULL != ptrShaParams))
    {
        status = DTHE_SHA_RETURN_SUCCESS;
    }

    if(status == DTHE_SHA_RETURN_SUCCESS)
    {
        config              = (DTHE_Config *) handle;
        attrs               = config->attrs;
        ptrShaRegs          = (CSL_EIP57T_SHARegs *)attrs->shaBaseAddr;
        dataLenBytes        = ptrShaParams->dataLenBytes;
        DTHE_SHA_setAlgorithm(ptrShaRegs, ptrShaParams->algoType);
        /* Sanity Check: The HMAC Key Size cannot be greater than the MAX allowed. */
        if (ptrShaParams->keySize > DTHE_HMAC_SHA_MAX_KEY_SIZE_BYTES)
        {
            /* Long HMAC Keys are not supported. */
            status = DTHE_SHA_RETURN_FAILURE;
        }

        if(status == DTHE_SHA_RETURN_SUCCESS)
        {
            /* Initialize the padded key */
            (void)memset ((void *)&hmacPaddedKey[0], 0, sizeof(hmacPaddedKey));

            /* Copy the key if one was provided: */
            if (ptrShaParams->keySize != 0U)
            {
                /* Copy the key data into the HMAC Padded Key: */
                (void)memcpy ((void *)&hmacPaddedKey[0], (const void*)ptrShaParams->ptrKey, ptrShaParams->keySize);
            }

            /* Which algorithm are we executing? */
            if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
            {
                /* SHA256: Outer & Inner Keys are 256bits = 32bytes = 8words */
                DTHE_SHA_setHMACOuterKey(ptrShaRegs, &hmacPaddedKey[0]);
                DTHE_SHA_setHMACInnerKey(ptrShaRegs, &hmacPaddedKey[8]);

                /* HMAC Processing:-
                *  - Algorithm Constants are not used
                *  - Compute the hash and close it here. */
                DTHE_SHA_setUseAlgoConstants(ptrShaRegs, 0U);
                DTHE_SHA_setCloseHash(ptrShaRegs, 1U);
                DTHE_SHA_setHMACKeyProcessing(ptrShaRegs, 1U);
                DTHE_SHA_setHMACOuterHash(ptrShaRegs, 1U);

                /* Set the data length: */
                DTHE_SHA_setHashLength(ptrShaRegs, dataLenBytes);

                /* SHA256: Setup the block & shift size: */
                blockSize = DTHE_SHA256_BLOCK_SIZE;
                shiftSize = DTHE_SHA256_SHIFT_SIZE;
            }
            else
            {
                /* SHA512: Outer & Inner Keys are 512bits = 64bytes = 16words */
                DTHE_SHA512_setHMACOuterKey(ptrShaRegs, &hmacPaddedKey[0]);
                DTHE_SHA512_setHMACInnerKey(ptrShaRegs, &hmacPaddedKey[16]);

                /* HMAC Processing:-
                *  - Algorithm Constants are not used
                *  - Compute the hash and close it here. */
                DTHE_SHA512_setUseAlgoConstants(ptrShaRegs, 0U);
                DTHE_SHA512_setCloseHash(ptrShaRegs, 1U);
                DTHE_SHA512_setHMACKeyProcessing(ptrShaRegs, 1U);
                DTHE_SHA512_setHMACOuterHash(ptrShaRegs, 1U);

                /* Set the data length: */
                DTHE_SHA512_setHashLength(ptrShaRegs, dataLenBytes);

                /* SHA512: Setup the block & shift size: */
                blockSize = DTHE_SHA512_BLOCK_SIZE;
                shiftSize = DTHE_SHA512_SHIFT_SIZE;
            }

            /* Determine the data length in words: */
            dataLenWords = dataLenBytes / 4U;

            /* Compute the number of blocks: */
            numBlocks = dataLenWords / blockSize;

            /* Compute the number of partial words which need to be handled seperately */
            numPartialWords = dataLenWords % blockSize;

            /* Compute the number of full blocks which need to be processed: */
            for (index = 0U; index < numBlocks; index = index + 1U)
            {
                /* Ensure that the SHA IP Block is ready to receive data: */
                DTHE_SHA_pollInputReady(ptrShaRegs);

                /* Write the data block: */
                DTHE_SHA_writeDataBlock(ptrShaRegs,
                                        &ptrShaParams->ptrDataBuffer[index << shiftSize],
                                        blockSize);

                /* Compute the number of bytes which have been processed: */
                numBytes = numBytes + (blockSize * sizeof(uint32_t));
            }

            /* Process any left over data: */
            if (numPartialWords != 0U)
            {
                /* Ensure that the SHA IP Block is ready to receive data: */
                DTHE_SHA_pollInputReady(ptrShaRegs);

                /* Write the data block: */
                DTHE_SHA_writeDataBlock(ptrShaRegs,
                                        &ptrShaParams->ptrDataBuffer[index << shiftSize],
                                        (uint8_t)numPartialWords);

                /* Compute the number of bytes which have been processed: */
                numBytes = numBytes + (numPartialWords * sizeof(uint32_t));
            }

             /* Do we need to account for some additional bytes? */
            if (dataLenBytes != numBytes)
            {
                /* Get the pointer to the data buffer in bytes which will be written: */
                ptrByteDataBuffer = (uint8_t*)ptrShaParams->ptrDataBuffer;

                /* Copy into the partial word: */
                (void)memcpy ((void *)&partialWord, (void *)&ptrByteDataBuffer[numBytes], (dataLenBytes - numBytes));

                /* Ensure that the SHA IP Block is ready to receive data: */
                DTHE_SHA_pollInputReady(ptrShaRegs);

                /* Write the data block: */
                DTHE_SHA_writeDataBlock(ptrShaRegs, &partialWord, 1U);
            }

            /* Poll till the intermediate hash results are available: */
            DTHE_SHA_pollOutputReady (ptrShaRegs);

            /* Get the digest count and value: */
            if (ptrShaParams->algoType == DTHE_SHA_ALGO_SHA256)
            {
                DTHE_SHA_getHashDigest(ptrShaRegs, &ptrShaParams->digest[0]);
                gDTHESHAdigestCount = DTHE_SHA_getDigestCount(ptrShaRegs);
            }
            else
            {
                DTHE_SHA512_getHashDigest(ptrShaRegs, &ptrShaParams->digest[0]);
                gDTHESHAdigestCount = DTHE_SHA512_getDigestCount(ptrShaRegs);
            }

            /* SHA Computation is in progress: */
            gDTHESHAInProgress = FALSE;
        }
    }
    return (status);
}
/* ========================================================================== */
/*                         Internal Function Definitions                      */
/* ========================================================================== */
/**
 *  \brief  The function is used to enable/disable the interrupts
 *
 *  \param  ptrSHARegs      Pointer to the EIP57T SHA Registers
 *
 *  \param  intStatus       Flag which is used to enable(1)/disable(0) the interrupts
 *
 */
static void DTHE_SHA_setInterruptStatus(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t intStatus)
{
    CSL_FINSR(ptrSHARegs->SYSCONFIG, 2U, 2U, intStatus);
    return;
}

/**
 *  \brief  The function is used to poll until the SHA IP block is ready to
 *          receive the context
 *
 *  \param  ptrSHARegs      Pointer to the SHA Driver Context Object
 *
 */
static void DTHE_SHA_pollContextReady(CSL_EIP57T_SHARegs* ptrSHARegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = DTHE_SHA_isContextReadyIRQ(ptrSHARegs);
    }
    return;
}

/**
 *  \brief  The function is used to get the context ready IRQ status
 *
 *  \param  ptrSHARegs      Pointer to the EIP57T SHA Registers
 *
 *  \retval                 1 - Available for a new context
 *                          0 - Not available for a new context
 */
static uint8_t DTHE_SHA_isContextReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs)
{
    return CSL_FEXTR (ptrSHARegs->IRQSTATUS, 3U, 3U);
}

/**
 *  \brief  The function is used to enable/disable the usage of the algorithm
 *      constants
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param useAlgConstants  Flag which will use enable(1)/disable(0) the usage of the algorithm constants
 *
 */
static void DTHE_SHA_setUseAlgoConstants(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t useAlgConstants)
{
    CSL_FINSR (ptrSHARegs->HASH_MODE, 3U, 3U, useAlgConstants);
    return;
}

/**
 *  \brief  The function is used to close/continue the hash algorithm
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param closeHashFlag    Flag which will close(1)/continue(0) the hash algorithm
 *
 */
static void DTHE_SHA_setCloseHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t closeHashFlag)
{
    CSL_FINSR (ptrSHARegs->HASH_MODE, 4U, 4U, closeHashFlag);
    return;
}

/**
 *  \brief The function is used to set the HMAC Key processing
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param hmacKeyFlag      Flag which will enable(1)/disable(0) the HMAC key processing
 *
 */
static void DTHE_SHA_setHMACKeyProcessing(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacKeyFlag)
{
    CSL_FINSR (ptrSHARegs->HASH_MODE, 5U, 5U, hmacKeyFlag);
    return;
}

/**
 *  \brief  The function is used to set the HMAC Outer Hash flag
 *
 *  \param  ptrSHARegs      Pointer to the EIP57T SHA Registers
 *  \param  hmacOuterHash   Flag which will enable(1)/disable(0) the HMAC outer hash is performed
 *                          on the hash digest when the inner hash has finished
 *
 */
static void DTHE_SHA_setHMACOuterHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacOuterHash)
{
    CSL_FINSR (ptrSHARegs->HASH_MODE, 7U, 7U, hmacOuterHash);
    return;
}

/**
 *  \brief  The function is used to enable/disable the usage of the algorithm
 *      constants
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param useAlgConstants  Flag which will use enable(1)/disable(0) the usage of the algorithm constants
 *
 */
static void DTHE_SHA512_setUseAlgoConstants(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t useAlgConstants)
{
    CSL_FINSR (ptrSHARegs->HASH512_MODE, 3U, 3U, useAlgConstants);
    return;
}

/**
 *  \brief  The function is used to close/continue the hash algorithm
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param closeHashFlag    Flag which will close(1)/continue(0) the hash algorithm
 *
 */
static void DTHE_SHA512_setCloseHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t closeHashFlag)
{
    CSL_FINSR (ptrSHARegs->HASH512_MODE, 4U, 4U, closeHashFlag);
    return;
}

/**
 *  \brief  The function is used to set the HMAC Key processing
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param hmacKeyFlag      Flag which will enable(1)/disable(0) the HMAC key processing
 *
 */
static void DTHE_SHA512_setHMACKeyProcessing(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacKeyFlag)
{
    CSL_FINSR (ptrSHARegs->HASH512_MODE, 5U, 5U, hmacKeyFlag);
    return;
}

/**
 *  \brief  The function is used to set the HMAC Outer Hash flag
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param hmacOuterHash    Flag which will enable(1)/disable(0) the HMAC outer hash is performed
 *                          on the hash digest when the inner hash has finished
 *
 */
static void DTHE_SHA512_setHMACOuterHash(CSL_EIP57T_SHARegs* ptrSHARegs, uint8_t hmacOuterHash)
{
    CSL_FINSR (ptrSHARegs->HASH_MODE, 7U, 7U, hmacOuterHash);
    return;
}

/**
 *  \brief  The function is used to set the length of the block to be processed
 *          for the hash operation
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param length           Length of the data
 *
 */
static void DTHE_SHA_setHashLength(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t length)
{
    ptrSHARegs->LENGTH = length;
    return;
}

/**
 *  \brief  The function is used to set the length of the block to be processed
 *          for the hash operation
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param length           Length of the data
 *
 */
static void DTHE_SHA512_setHashLength(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t length)
{
    ptrSHARegs->HASH512_LENGTH = length;
    return;
}

/**
 *  \brief  The function is used to poll until the SHA IP block is ready to
 *          receive data
 *
 *  \param ptrSHARegs       Pointer to the SHA Driver Context Object
 *
 */
static void DTHE_SHA_pollInputReady(CSL_EIP57T_SHARegs* ptrSHARegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = DTHE_SHA_isInputReadyIRQ(ptrSHARegs);
    }
    return;
}

/**
 *  \brief  The function is used write the data block. Data Block sizes
 *          are specific to the size of algorithm selected.
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param ptrDataBlock     Pointer to the data block to be written
 *  \param blockSize        Block Size. This is selected on the basis of the algorithm.
 *
 */
static void DTHE_SHA_writeDataBlock(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrDataBlock, uint8_t blockSize)
{
    uint8_t index;
    for (index = 0U; index < blockSize; index = index + 1U)
    {
        ptrSHARegs->DATA_IN[0] = ptrDataBlock[index];
    }
    return;
}

/**
 *  \brief  The function is used to poll until the SHA IP block is ready with
 *          the results
 *
 *  \param ptrSHARegs       Pointer to the SHA Driver Context Object
 *
 */
static void DTHE_SHA_pollOutputReady (CSL_EIP57T_SHARegs* ptrSHARegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = DTHE_SHA_isOutputReadyIRQ(ptrSHARegs);
    }
    return;
}

/**
 *  \brief  The function is used to get the hash inner digest for SHA-256.
 *          The SHA Digest in this case is 32bytes.
 *
 *  \param  ptrSHARegs      Pointer to the EIP57T SHA Registers
 *  \param  ptrDigest       Pointer to the digest populated by the API
 *
 */
static void DTHE_SHA_getHashDigest(const CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t* ptrDigest)
{
    uint8_t index;
    for (index = 0U; index < 8U; index++)
    {
        ptrDigest[index] = ptrSHARegs->IDIGEST[index];
    }
    return;
}

/**
 *  \brief  The function is used to get the hash inner digest for SHA512.
 *          The SHA Digest in this case is 64bytes.
 *
 *  \param  ptrSHARegs      Pointer to the EIP57T SHA Registers
 *  \param  ptrDigest       Pointer to the digest populated by the API
 *
 */
static void DTHE_SHA512_getHashDigest(const CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t* ptrDigest)
{
    uint8_t index;
    for (index = 0U; index < 16U; index++)
    {
        ptrDigest[index] = ptrSHARegs->HASH512_IDIGEST[index];
    }
    return;
}

/**
 *  \brief  The function is used to set the selected algorithm
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param algorithm        Algorithm to be executed
 *
 */
static void DTHE_SHA_setAlgorithm(CSL_EIP57T_SHARegs* ptrSHARegs, uint32_t algorithm)
{
    uint8_t     value;

    switch (algorithm)
    {
        case CSL_EIP57T_SHAAlgo_MD5:
        {
            value = 0U;
            break;
        }
        case CSL_EIP57T_SHAAlgo_SHA384:
        {
            value = 1U;
            break;
        }
        case CSL_EIP57T_SHAAlgo_SHA1:
        {
            value = 2U;
            break;
        }
        case CSL_EIP57T_SHAAlgo_SHA512:
        {
            value = 3U;
            break;
        }
        case CSL_EIP57T_SHAAlgo_SHA224:
        {
            value = 4U;
            break;
        }
        default:
        {
            value = 6U;
            break;
        }
    }
    CSL_FINSR (ptrSHARegs->HASH_MODE, 2U, 0U, value);
    return;
}

/**
 *  \brief The function is used to get the output ready IRQ status
 *
 *  \param ptrSHARegs Pointer to the EIP57T SHA Registers
 *
 *  \return 1 - (Partial) result is available
 *          0 - (Partial) result is not available
 */
static uint8_t DTHE_SHA_isOutputReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs)
{
    return CSL_FEXTR (ptrSHARegs->IRQSTATUS, 0U, 0U);
}

/**
 *  \brief The function is used to get the input ready IRQ status
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *
 *  \return 1 - Data FIFO is ready to receive data
 *          0 - Data FIFO is not ready to receive data
 */
static uint8_t DTHE_SHA_isInputReadyIRQ(const CSL_EIP57T_SHARegs* ptrSHARegs)
{
    return CSL_FEXTR (ptrSHARegs->IRQSTATUS, 1U, 1U);
}

/**
 *  \brief The function is used to get the digest count for the SHA512
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *
 *  \retval                 Digest Count
 */
static uint32_t DTHE_SHA512_getDigestCount(const CSL_EIP57T_SHARegs* ptrSHARegs)
{
    return ptrSHARegs->HASH512_DIGEST_COUNT;
}

/**
 *  \brief The function is used to get the digest count
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *
 *  \retval                 Digest Count
 */
static uint32_t DTHE_SHA_getDigestCount(const CSL_EIP57T_SHARegs* ptrSHARegs)
{
    return ptrSHARegs->DIGEST_COUNT;
}

/**
 *  \brief  The function is used to write the lower 256bits (32bytes) of the HMAC Key
 *          to the Outer Digest. Padding is done outside this function.
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param ptrHMACKey       Pointer to the lower 256bits to be configured
 *
 */
static void DTHE_SHA_setHMACOuterKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey)
{
    uint8_t index;
    for (index = 0U; index < 8U; index = index + 1U)
    {
        ptrSHARegs->ODIGEST[index] = ptrHMACKey[index];
    }
    return;
}

/**
 *  \brief The function is used to write the upper 256bits (32bytes) of the HMAC Key
 *         to the Inner Digest.
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param ptrHMACKey       Pointer to the upper 256bits to be configured
 *
 */
static void DTHE_SHA_setHMACInnerKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey)
{
    uint8_t index;
    for (index = 0U; index < 8U; index = index + 1U)
    {
        ptrSHARegs->IDIGEST[index] = ptrHMACKey[index];
    }
    return;
}

/**
 *  \brief The function is used to write the lower 512bits (64bytes) of the HMAC Key
 *         to the Outer Digest. Padding is done outside this function.
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param ptrHMACKey       Pointer to the lower 512bits to be configured
 *
 */
static void DTHE_SHA512_setHMACOuterKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey)
{
    uint8_t index;
    for (index = 0U; index < 16U; index = index + 1U)
    {
        ptrSHARegs->HASH512_ODIGEST[index] = ptrHMACKey[index];
    }
    return;
}

/**
 *  \brief The function is used to write the upper 512bits (64bytes) of the HMAC Key
 *         to the Inner Digest.
 *
 *  \param ptrSHARegs       Pointer to the EIP57T SHA Registers
 *  \param ptrHMACKey       Pointer to the upper 512bits to be configured
 *
 */
static void DTHE_SHA512_setHMACInnerKey(CSL_EIP57T_SHARegs* ptrSHARegs, const uint32_t* ptrHMACKey)
{
    uint8_t index;
    for (index = 0U; index < 16U; index = index + 1U)
    {
        ptrSHARegs->HASH512_IDIGEST[index] = ptrHMACKey[index];
    }
    return;
}
