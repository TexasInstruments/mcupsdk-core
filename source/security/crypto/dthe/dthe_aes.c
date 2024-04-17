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
 *  \file   dthe_aes.c
 *
 *  \brief  This file contains the implementation of Dthe aes driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <security/crypto/dthe/dthe_aes.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** This is the max value of datasize taken in case of streaming mode */
#define     MAX_VALUE                   (0xFFFFFFFFU)

/** This is the state of on ongoing stream state */
#define     AES_STATE_NEW               (0x00U)

/** This is the state of a stream in progress */
#define     AES_STATE_IN_PROGRESS       (0xA5U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

static uint8_t gStreamState = AES_STATE_NEW;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void DTHE_AES_setDMAContextStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus);
static void DTHE_AES_setDMAOutputRequestStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus);
static void DTHE_AES_setDMAInputRequestStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus);
static void DTHE_AES_setKeySize(CSL_AesRegs *ptrAesRegs, uint8_t size);
static void DTHE_AES_set256BitKey1(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey);
static void DTHE_AES_clearIV(CSL_AesRegs *ptrAesRegs);
static void DTHE_AES_setIV(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrIV);
static void DTHE_AES_set128BitKey2Part1(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey);
static void DTHE_AES_set128BitKey2Part2(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey);
static void DTHE_AES_pollInputReady(CSL_AesRegs *ptrAesRegs);
static void DTHE_AES_writeDataBlock(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrData);
static void DTHE_AES_pollOutputReady(CSL_AesRegs *ptrAesRegs);
static void DTHE_AES_pollContextReady(CSL_AesRegs *ptrAesRegs);
static void DTHE_AES_readDataBlock(CSL_AesRegs *ptrAesRegs, uint32_t* ptrData);
static void DTHE_AES_resetModule(CSL_AesRegs *ptrAesRegs);
static void DTHE_AES_controlMode(CSL_AesRegs *ptrAesRegs, uint32_t algoType);
static void DTHE_AES_setOpType(CSL_AesRegs *ptrAesRegs, uint32_t opType);
static void DTHE_AES_setDataLengthBytes(CSL_AesRegs *ptrAesRegs, uint32_t dataLenBytes);
static void DTHE_AES_readTag(CSL_AesRegs *ptrAesRegs, uint32_t* ptrTag);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void DTHE_AES_resetModule(CSL_AesRegs *ptrAesRegs)
{
	/* Reset the AES and ensure it is out of reset */
    CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG, AES_S_SYSCONFIG_SOFTRESET, 1U);
    /* TODO: Timeout implementation */
    while (CSL_REG32_FEXT(&ptrAesRegs->SYSSTS, AES_S_SYSSTS_RESETDONE) == 0U)
    {
    }
}

static void DTHE_AES_controlMode(CSL_AesRegs *ptrAesRegs, uint32_t algoType)
{
	if(algoType == DTHE_AES_ECB_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_CBC_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_CBC);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_CTR_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_CTR);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_ICM_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_ICM);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_CFB_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_CFB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_F8_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_F8);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_F9_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F9_F9);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if(algoType == DTHE_AES_XTS_MODE)
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_RESETVAL);
    }
    else if((algoType == DTHE_AES_CBC_MAC_MODE)||(algoType == DTHE_AES_CMAC_MODE))
    {
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CBCMAC, CSL_AES_S_CTRL_CBCMAC_CBCMAC);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_MODE, CSL_AES_S_CTRL_MODE_ECB);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR, CSL_AES_S_CTRL_CTR_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_ICM, CSL_AES_S_CTRL_ICM_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CFB, CSL_AES_S_CTRL_CFB_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F8, CSL_AES_S_CTRL_F8_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_F9, CSL_AES_S_CTRL_F9_RESETVAL);
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_XTS, CSL_AES_S_CTRL_XTS_NOOP);
    }
}

static void DTHE_AES_CTRWidth(CSL_AesRegs *ptrAesRegs, uint32_t ctrWidth)
{
    uint32_t ctrWidthRegValue = 3U;

	if(ctrWidth == DTHE_AES_CTR_WIDTH_32)
    {
        ctrWidthRegValue = CSL_AES_S_CTRL_CTR_WIDTH_COUNTER32;
    }
    else if(ctrWidth == DTHE_AES_CTR_WIDTH_64)
    {
        ctrWidthRegValue = CSL_AES_S_CTRL_CTR_WIDTH_COUNTER64;
    }
    else if(ctrWidth == DTHE_AES_CTR_WIDTH_96)
    {
        ctrWidthRegValue = CSL_AES_S_CTRL_CTR_WIDTH_COUNTER96;
    }
    else if(ctrWidth == DTHE_AES_CTR_WIDTH_128)
    {
        ctrWidthRegValue = CSL_AES_S_CTRL_CTR_WIDTH_COUNTER128;
    }

    CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_CTR_WIDTH, ctrWidthRegValue);
}

static void DTHE_AES_setOpType(CSL_AesRegs *ptrAesRegs, uint32_t opType)
{
	/* Operation Mode: Encryption/Decryption */
   if (opType == DTHE_AES_ENCRYPT)
   {
       /* Encryption: */
       CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_DIRECTION, 1U);
   }
   else
   {
       /* Decryption: */
       CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_DIRECTION, 0U);
   }

}

static void DTHE_AES_setDataLengthBytes(CSL_AesRegs *ptrAesRegs, uint32_t dataLenBytes)
{
	/* Setup the data length: */
    CSL_REG32_FINS(&ptrAesRegs->C_LENGTH_0, AES_S_C_LENGTH_0_LENGTH, dataLenBytes);
}


DTHE_AES_Return_t DTHE_AES_open(DTHE_Handle handle)
{
    DTHE_AES_Return_t status  = DTHE_AES_RETURN_FAILURE;
    DTHE_Config       *config = NULL;
    DTHE_Attrs        *attrs  = NULL;
    CSL_AesRegs       *ptrAesRegs;

    if(NULL != handle)
    {
        status  = DTHE_AES_RETURN_SUCCESS;
    }

    if(status  == DTHE_AES_RETURN_SUCCESS)
    {
        config          = (DTHE_Config *) handle;
        attrs           = config->attrs;
        ptrAesRegs      = (CSL_AesRegs *)attrs->aesBaseAddr;

        gStreamState = AES_STATE_NEW;

        /* Soft-Reset AES Module */
		DTHE_AES_resetModule(ptrAesRegs);

        /* Disable the DMA for the AES */
        DTHE_AES_setDMAContextStatus(ptrAesRegs, 0);
        DTHE_AES_setDMAOutputRequestStatus(ptrAesRegs, 0);
        DTHE_AES_setDMAInputRequestStatus(ptrAesRegs, 0);

        /* Disable Save Context */
        CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_SAVE_CONTEXT, 0U);
    }

    return (status);
}

DTHE_AES_Return_t DTHE_AES_execute(DTHE_Handle handle, const DTHE_AES_Params* ptrParams)
{
    DTHE_AES_Return_t status  = DTHE_AES_RETURN_FAILURE;
    DTHE_Config       *config = NULL;
    DTHE_Attrs        *attrs  = NULL;
    CSL_AesRegs     *ptrAesRegs;
    uint32_t*       ptrWordInputBuffer;
    uint32_t*       ptrWordOutputBuffer;
    uint32_t        dataLenWords;
    uint32_t        numBlocks;
    uint32_t        partialDataSize;
    uint32_t        index;
    uint32_t        numBytes = 0U;
    uint8_t         inPartialBlock[32U];
    uint8_t         outPartialBlock[32U];

    if (NULL != handle)
    {
        status  = DTHE_AES_RETURN_SUCCESS;
    }
    if (status  == DTHE_AES_RETURN_SUCCESS)
    {
        config          = (DTHE_Config *) handle;
        attrs           = config->attrs;
        ptrAesRegs      = (CSL_AesRegs *)attrs->aesBaseAddr;

        /* This flow is for One-Shot mode and Stream Mode as INIT only */
        if(((ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)||(ptrParams->streamState == DTHE_AES_STREAM_INIT))&&\
            (gStreamState == AES_STATE_NEW))
        {
            DTHE_AES_controlMode(ptrAesRegs, ptrParams->algoType);

            /* Key Size setting */
            DTHE_AES_setKeySize(ptrAesRegs, ptrParams->keyLen);

            if((ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)&&(ptrParams->dataLenBytes == 0))
            {
                status = DTHE_AES_RETURN_FAILURE;
            }

            /* Sanity Check: For Decryption data length always needs to be aligned */
            if (ptrParams->opType == DTHE_AES_DECRYPT)
            {
                if ((ptrParams->dataLenBytes % 4U) != 0U)
                {
                    status = DTHE_AES_RETURN_FAILURE;
                }
            }

            /* Sanity Check: Key Validation */
            if (status == DTHE_AES_RETURN_SUCCESS)
            {
                /* KEK Mode or Normal Key Mode: */
                if (ptrParams->useKEKMode == TRUE)
                {
                    /* If KEKMode is set, configure Muxes for KEK
                    * to be passed to AES Engine */
                    /* KEK Mode: Enable Direct Bus */
                    CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DIRECTBUSEN,1U);
                }
                else
                {
                    /* Normal Mode: Key should always be specified */
                    if (ptrParams->ptrKey == NULL)
                    {
                        status = DTHE_AES_RETURN_FAILURE;
                    }
                }

                if((ptrParams->algoType == DTHE_AES_CBC_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_CTR_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_ICM_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_CFB_MODE))
                {
                    if (ptrParams->ptrIV == NULL)
                    {
                        status = DTHE_AES_RETURN_FAILURE;
                    }
                }

                /* Select the key input: */
                if (ptrParams->useKEKMode == FALSE)
                {
                    /* Normal Key Mode: */
                    CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DIRECTBUSEN,0U);

                    /* Normal Mode: Key should always be specified */
                    if (ptrParams->ptrKey != NULL)
                    {
                        /* Configure the key which is to be used: */
                        DTHE_AES_set256BitKey1 (ptrAesRegs, ptrParams->ptrKey);
                    }

                    if (ptrParams->algoType == DTHE_AES_CMAC_MODE)
                    {
                        DTHE_AES_set128BitKey2Part1(ptrAesRegs, ptrParams->ptrKey1);
                        DTHE_AES_set128BitKey2Part2(ptrAesRegs, ptrParams->ptrKey2);
                    }
                }

                DTHE_AES_setOpType(ptrAesRegs, ptrParams->opType);

                if((ptrParams->algoType == DTHE_AES_CTR_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_ICM_MODE))
                {
                    DTHE_AES_CTRWidth(ptrAesRegs, ptrParams->counterWidth);
                }

                /* Configure the Initialization Vector */
                if((ptrParams->algoType == DTHE_AES_CBC_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_CTR_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_ICM_MODE)\
                    ||(ptrParams->algoType == DTHE_AES_CFB_MODE))
                {
                    if (ptrParams->ptrIV != NULL)
                    {
                        DTHE_AES_setIV(ptrAesRegs, ptrParams->ptrIV);
                    }
                }
                else if((ptrParams->algoType == DTHE_AES_CBC_MAC_MODE)||(ptrParams->algoType == DTHE_AES_CMAC_MODE))
                {
                    /* Clear the IV value */
                    DTHE_AES_clearIV(ptrAesRegs);
                    /* Enable Save Context in CTRL register*/
                    CSL_REG32_FINS(&ptrAesRegs->CTRL, AES_S_CTRL_SAVE_CONTEXT, 1U);
                }

                /*
                - DataLength is sent by user, then set the same here.
                - DataLength is not sent by user, then set the length as maximum. */
                if((ptrParams->streamState != DTHE_AES_ONE_SHOT_SUPPORT)&&(ptrParams->dataLenBytes == 0))
                {
                    /* Setup the data length as 0xFFFFFFFF */
                    DTHE_AES_setDataLengthBytes(ptrAesRegs, MAX_VALUE);
                }
                else
                {
                    /* Setup the data length: */
                    DTHE_AES_setDataLengthBytes(ptrAesRegs, ptrParams->dataLenBytes);
                }

                gStreamState = AES_STATE_IN_PROGRESS;
            }
        }
        /* Stream Mode Update should support streamSize aligned to 16B only */
        else if((gStreamState == AES_STATE_IN_PROGRESS)&&\
                ((ptrParams->streamState == DTHE_AES_STREAM_UPDATE)||(ptrParams->streamState == DTHE_AES_STREAM_FINISH)))
        {
            if ((ptrParams->streamState == DTHE_AES_STREAM_UPDATE)&&((ptrParams->streamSize % 16U) != 0U))
            {
                status = DTHE_AES_RETURN_FAILURE;
            }
        }
        else
        {
            status = DTHE_AES_RETURN_FAILURE;
        }

        /* Execute the AES Driver: */
        if ((status == DTHE_AES_RETURN_SUCCESS)&&(gStreamState == AES_STATE_IN_PROGRESS))
        {
            /* This flow is for one-shot in continuation to the above flow
               In case of Update and Finish start execution from here */
            if((ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)||(ptrParams->streamState == DTHE_AES_STREAM_UPDATE)||(ptrParams->streamState == DTHE_AES_STREAM_FINISH))
            {
                if((ptrParams->streamState == DTHE_AES_STREAM_FINISH)&&(ptrParams->dataLenBytes == 0U))
                {
                    /* Setup the data length: */
                    DTHE_AES_setDataLengthBytes(ptrAesRegs,  ptrParams->streamSize);
                }

                /* Setup the input & output: */
                if (ptrParams->opType == DTHE_AES_ENCRYPT)
                {
                    /* Encryption: Plain Text is the Input & Encrypted Data is the Output */
                    ptrWordInputBuffer  = &ptrParams->ptrPlainTextData[0];
                    ptrWordOutputBuffer = &ptrParams->ptrEncryptedData[0];
                }
                else
                {
                    /* Decryption: Encrypted Data is the Input & Plain Text is the Output */
                    ptrWordInputBuffer  = &ptrParams->ptrEncryptedData[0];
                    ptrWordOutputBuffer = &ptrParams->ptrPlainTextData[0];
                }

                if(ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)
                {
                    /* Determine the data length in words: */
                    dataLenWords = ptrParams->dataLenBytes / 4U;
                    /* Compute the number of bytes which need to be handled seperately */
                    partialDataSize = ptrParams->dataLenBytes % 16U;
                }
                else
                {
                    /* Determine the data length in words: */
                    dataLenWords = ptrParams->streamSize / 4U;
                    /* Compute the number of bytes which need to be handled seperately */
                    partialDataSize = ptrParams->streamSize % 16U;
                }

                /* Compute the number of full blocks which can be written: Each block is 4words long*/
                numBlocks = (dataLenWords / 4U);

                /* Cycle through and write all the full blocks: */
                for (index = 0U; index < numBlocks; index++)
                {
                    /* Wait for the AES IP to be ready to receive the data: */
                    DTHE_AES_pollInputReady(ptrAesRegs);

                    /* Write the data: */
                    DTHE_AES_writeDataBlock(ptrAesRegs, &ptrWordInputBuffer[index << 2U]);

                    if((ptrParams->algoType != DTHE_AES_CBC_MAC_MODE)&&(ptrParams->algoType != DTHE_AES_CMAC_MODE))
                    {
                        /* Wait for the AES IP to be ready with the output data */
                        DTHE_AES_pollOutputReady(ptrAesRegs);

                        /* Read the decrypted data into the decrypted block: */
                        DTHE_AES_readDataBlock(ptrAesRegs, &ptrWordOutputBuffer[index << 2U]);
                    }

                    /* Compute the number of bytes which have been processed: */
                    numBytes = numBytes + (4U * sizeof(uint32_t));
                }

                /* - This flow is for one-shot in continuation to the above flow
                   - In case of Finish continue execution from here
                   - Update should not execute this because this is for partial block
                   handling which is not supported by Update CALL */
                if((ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)||(ptrParams->streamState == DTHE_AES_STREAM_FINISH))
                {
                    /* Process any left over data: */
                    if(partialDataSize != 0U)
                    {
                        /* Initialize the partial block: */
                        (void)memset ((void *)&inPartialBlock, 0, sizeof(inPartialBlock));
                        (void)memset ((void *)&outPartialBlock, 0, sizeof(outPartialBlock));

                        /* Copy the data into the partial block: */
                        (void)memcpy ((void *)&inPartialBlock,
                                (void *)&ptrWordInputBuffer[index << 2U],
                                partialDataSize);

                        if (ptrParams->algoType == DTHE_AES_CMAC_MODE)
                        {
                            inPartialBlock[partialDataSize] = 0x80;
                        }

                        /* Wait for the AES IP to be ready to receive the data: */
                        DTHE_AES_pollInputReady(ptrAesRegs);

                        /* Write the data: */
                        DTHE_AES_writeDataBlock(ptrAesRegs, (uint32_t *)&inPartialBlock[0U]);

                        if((ptrParams->algoType != DTHE_AES_CBC_MAC_MODE)&&(ptrParams->algoType != DTHE_AES_CMAC_MODE))
                        {
                            /* Wait for the AES IP to be ready with the output data */
                            DTHE_AES_pollOutputReady(ptrAesRegs);

                            /* Read the decrypted data into the decrypted block: */
                            DTHE_AES_readDataBlock(ptrAesRegs, (uint32_t *)&outPartialBlock[0U]);

                            if((ptrParams->algoType == DTHE_AES_ECB_MODE)||(ptrParams->algoType == DTHE_AES_CBC_MODE))
                            {
                                /* Copy the data into the output buffer, always is going to be 16U */
                                (void)memcpy ((void *)&ptrWordOutputBuffer[index << 2U],
                                        (void *)&outPartialBlock[0U],
                                        16U);
                            }
                            else
                            {
                                /* Copy the data into the output buffer, always is going to be 16U */
                                (void)memcpy ((void *)&ptrWordOutputBuffer[index << 2U],
                                        (void *)&outPartialBlock[0U],
                                        partialDataSize);
                            }
                        }

                        /* Compute the number of bytes which have been processed: */
                        numBytes = numBytes + partialDataSize;
                    }

                    if((ptrParams->algoType == DTHE_AES_CBC_MAC_MODE)||(ptrParams->algoType == DTHE_AES_CMAC_MODE))
                    {
                        DTHE_AES_pollContextReady(ptrAesRegs);
                        DTHE_AES_readTag(ptrAesRegs, &ptrParams->ptrTag[0]);
                    }
                }

                if(ptrParams->streamState == DTHE_AES_ONE_SHOT_SUPPORT)
                {
                    if(numBytes != ptrParams->dataLenBytes)
                    {
                        status = DTHE_AES_RETURN_FAILURE;
                    }

                    gStreamState = AES_STATE_NEW;
                }
                else if(ptrParams->streamState == DTHE_AES_STREAM_FINISH)
                {
                    gStreamState = AES_STATE_NEW;
                }
            }
        }
    }
    return (status);
}

DTHE_AES_Return_t DTHE_AES_close(DTHE_Handle handle)
{
    DTHE_AES_Return_t  status  = DTHE_AES_RETURN_FAILURE;
    DTHE_Config        *config = NULL;
    DTHE_Attrs         *attrs  = NULL;
    CSL_AesRegs        *ptrAesRegs;

    if(NULL != handle)
    {
        status  = DTHE_AES_RETURN_SUCCESS;
    }
    if(status  == DTHE_AES_RETURN_SUCCESS)
    {
        config          = (DTHE_Config *) handle;
        attrs           = config->attrs;
        ptrAesRegs      = (CSL_AesRegs *)attrs->aesBaseAddr;

        DTHE_AES_resetModule(ptrAesRegs);

    }
    return (status);
}

/* ========================================================================== */
/*                 Internal Function Definations                              */
/* ========================================================================== */
/**
 * \brief                   The function is used to set the DMA context (input and output) status of the AES module.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   dmaStatus       Flag which is used to enable(1)/disable(0) the DMA status.
 *
 */
static void DTHE_AES_setDMAContextStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus)
{
    if (dmaStatus == 1U)
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG, AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN, 1U);
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG, AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN, 1U);
    }
    else
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG, AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN, 0U);
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG, AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN, 0U);
    }

    return;
}

/**
 * \brief                   The function is used to set the DMA Output request status
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   dmaStatus       Flag which is used to enable(1)/disable(0) the DMA status.
 *
 */
static void DTHE_AES_setDMAOutputRequestStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus)
{
    if (dmaStatus == 1U)
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN, 1U);
    }
    else
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN, 0U);
    }

    return;
}

/**
 * \brief                   The function is used to set the DMA Input request status
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   dmaStatus       Flag which is used to enable(1)/disable(0) the DMA status.
 *
 */
static void DTHE_AES_setDMAInputRequestStatus(CSL_AesRegs *ptrAesRegs, uint8_t dmaStatus)
{
    if (dmaStatus == 1U)
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN, 1U);
    }
    else
    {
        CSL_REG32_FINS(&ptrAesRegs->SYSCONFIG,AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN, 0U);
    }

    return;
}

/**
 * \brief                   The function is used to set the key size
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   size            Size of the key to be configured.
 *
 */
static void DTHE_AES_setKeySize(CSL_AesRegs *ptrAesRegs, uint8_t size)
{
    uint8_t keySize;

    if (size == CSL_AES_S_CTRL_KEY_SIZE_KEY128)
    {
        keySize = 1U;
    }
    else if (size == CSL_AES_S_CTRL_KEY_SIZE_KEY192)
    {
        keySize = 2U;
    }
    else if (size == CSL_AES_S_CTRL_KEY_SIZE_KEY256)
    {
        keySize = 3U;
    }
    else
    {
        keySize = 0U;
    }
    CSL_REG32_FINS(&ptrAesRegs->CTRL,AES_S_CTRL_KEY_SIZE, keySize);

    return;
}

/**
 * \brief                   The function is used to configure the key in the AES module This will only configure the 256bit keys.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   ptrKey          Pointer to the 256bit key to be used.
 *
 */
static void DTHE_AES_set256BitKey1(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey)
{
    ptrAesRegs->KEY1_0 = ptrKey[0U];
    ptrAesRegs->KEY1_1 = ptrKey[1U];
    ptrAesRegs->KEY1_2 = ptrKey[2U];
    ptrAesRegs->KEY1_3 = ptrKey[3U];
    ptrAesRegs->KEY1_4 = ptrKey[4U];
    ptrAesRegs->KEY1_5 = ptrKey[5U];
    ptrAesRegs->KEY1_6 = ptrKey[6U];
    ptrAesRegs->KEY1_7 = ptrKey[7U];

    return;
}

/**
 * \brief                   The function is used to configure the key in the AES module This will only configure the 128bit keys.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   ptrKey          Pointer to the 128bit key to be used.
 *
 */
static void DTHE_AES_set128BitKey2Part1(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey)
{
    ptrAesRegs->KEY2_0 = ptrKey[0U];
    ptrAesRegs->KEY2_1 = ptrKey[1U];
    ptrAesRegs->KEY2_2 = ptrKey[2U];
    ptrAesRegs->KEY2_3 = ptrKey[3U];

    return;
}

/**
 * \brief                   The function is used to configure the key in the AES module This will only configure the 128bit keys.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   ptrKey          Pointer to the 128bit key to be used.
 *
 */
static void DTHE_AES_set128BitKey2Part2(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrKey)
{
    ptrAesRegs->KEY2_4 = ptrKey[0U];
    ptrAesRegs->KEY2_5 = ptrKey[1U];
    ptrAesRegs->KEY2_6 = ptrKey[2U];
    ptrAesRegs->KEY2_7 = ptrKey[3U];

    return;
}

/**
 * \brief                   The function is used to set the Initialization Vector (IV) in the AES module.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers.
 *
 * \param   ptrIV           Pointer to the IV data to be used.
 *
 */
static void DTHE_AES_clearIV(CSL_AesRegs *ptrAesRegs)
{
    ptrAesRegs->IV_IN_0 = 0U;
    ptrAesRegs->IV_IN_1 = 0U;
    ptrAesRegs->IV_IN_2 = 0U;
    ptrAesRegs->IV_IN_3 = 0U;

    return;
}

/**
 * \brief                   The function is used to clear the Initialization Vector (IV) in the AES module.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers.
 *
 */
static void DTHE_AES_setIV(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrIV)
{
    /* Clear the IV value in registers */
    DTHE_AES_clearIV(ptrAesRegs);

    ptrAesRegs->IV_IN_0 = ptrIV[0U];
    ptrAesRegs->IV_IN_1 = ptrIV[1U];
    ptrAesRegs->IV_IN_2 = ptrIV[2U];
    ptrAesRegs->IV_IN_3 = ptrIV[3U];

    return;
}

/**
 * \brief                   The function is used to poll until the AES IP block is ready to receive data.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 */
static void DTHE_AES_pollInputReady(CSL_AesRegs *ptrAesRegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = CSL_REG32_FEXT(&ptrAesRegs->CTRL, AES_S_CTRL_INPUT_READY);
    }
    return;
}

/**
 * \brief                   The function is used to write the data and pass this to the AES engine.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers.
 *
 * \param   ptrData         Pointer to the data to be written.
 *
 */
static void DTHE_AES_writeDataBlock(CSL_AesRegs *ptrAesRegs, const uint32_t* ptrData)
{
    ptrAesRegs->DATA_IN_3 = ptrData[0U];
    ptrAesRegs->DATA_IN_2 = ptrData[1U];
    ptrAesRegs->DATA_IN_1 = ptrData[2U];
    ptrAesRegs->DATA_IN_0 = ptrData[3U];

    return;
}

/**
 * \brief                   The function is used to poll until the AES IP block has available data which can be read out.
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 */
static void DTHE_AES_pollOutputReady(CSL_AesRegs *ptrAesRegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = CSL_REG32_FEXT(&ptrAesRegs->CTRL, AES_S_CTRL_OUTPUT_READY);
    }
    return;
}

/**
 * \brief                   The function is used to poll until the AES IP block has available context out register
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 */
static void DTHE_AES_pollContextReady(CSL_AesRegs *ptrAesRegs)
{
    uint8_t     done = 0U;

    /* Loop around till the condition is met: */
    while (done == 0U)
    {
        done = CSL_REG32_FEXT(&ptrAesRegs->CTRL, AES_S_CTRL_SAVE_CONTEXT_READY);
    }
    return;
}

/**
 * \brief                   The function is used to read the data from the AES engine
 *
 * \param   ptrAesRegs      Pointer to the EIP38T AES Registers
 *
 * \param   ptrData         Pointer to the data buffer populated by the API
 *
 */
static void DTHE_AES_readDataBlock(CSL_AesRegs *ptrAesRegs, uint32_t* ptrData)
{
    ptrData[0] = ptrAesRegs->DATA_IN_3;
    ptrData[1] = ptrAesRegs->DATA_IN_2;
    ptrData[2] = ptrAesRegs->DATA_IN_1;
    ptrData[3] = ptrAesRegs->DATA_IN_0;

    return;
}

static void DTHE_AES_readTag(CSL_AesRegs *ptrAesRegs, uint32_t* ptrTag)
{
    ptrTag[0] = ptrAesRegs->TAG_OUT_0;
    ptrTag[1] = ptrAesRegs->TAG_OUT_1;
    ptrTag[2] = ptrAesRegs->TAG_OUT_2;
    ptrTag[3] = ptrAesRegs->TAG_OUT_3;

    return;
}
