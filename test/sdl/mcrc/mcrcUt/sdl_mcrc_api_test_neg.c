/* Copyright (c) 2021 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     sdl_mcrc_api_test_neg.c
 *
 *  \brief    This file contains mcrc API unit test code..
 *
 *  \details  mcrc unit tests
 **/

#include "mcrc_main.h"

int32_t sdl_mcrc_negTest(void)
{
    int32_t                       testStatus = SDL_APP_TEST_PASS;
#if defined (SOC_AM64X) || defined (SOC_AM243X)
	SDL_MCRC_InstType             instance = MCU_MCRC64_0 ;
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_MCRC_InstType             instance = MCRC0;
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    SDL_MCRC_InstType             instance = MCRC_INSTANCE;
#endif
    SDL_MCRC_Channel_t            channel = SDL_MCRC_CHANNEL_1;
    uint32_t                      watchdogPreload = MCRC_WATCHDOG_PRELOAD;
    uint32_t                      blockPreload = MCRC_BLOCK_PRELOAD;
    SDL_MCRC_ModeType             mode = SDL_MCRC_OPERATION_MODE_AUTO;
    SDL_MCRC_SignatureRegAddr_t pMCRCregAddr;
    SDL_MCRC_Signature_t         pCurPSASig, pPSAsig,pPSAsign;
    uint32_t                     pCurSecNum, pBusyFlag;
    uint32_t                     IntrMask = 1U;
    uint32_t                     pIntrstatus;
    SDL_MCRC_Signature_t         pSecSign;
    SDL_MCRC_Signature_t         pSeedSign;
                                pSeedSign.regL    = 255U;
                                pSeedSign.regH    = 255U;
    uint32_t                     patternCount = 255U;
    uint32_t                     sectorCount = 255U;
    SDL_MCRC_DataConfig_t         pDataConfig;

    /*  Error/Fualt test of init API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(SDL_MCRC_INVALID, channel, watchdogPreload, blockPreload)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(instance, 5U, watchdogPreload, blockPreload)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(instance, channel, SDL_MCRC_WDTOPLD_MAX+1U, blockPreload)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(instance, channel, SDL_MCRC_WDTOPLD_MAX+1U, blockPreload)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(instance, channel, watchdogPreload, SDL_MCRC_BCTOPLD_MAX+1U)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_init(instance, channel, watchdogPreload, SDL_MCRC_BCTOPLD_MAX+1U)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /* ---------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of verify init API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(SDL_MCRC_INVALID, channel, watchdogPreload, blockPreload) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(instance, 5U, watchdogPreload, blockPreload) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(instance, channel, SDL_MCRC_WDTOPLD_MAX+1U, blockPreload) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(instance, channel, watchdogPreload, SDL_MCRC_BCTOPLD_MAX+1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(SDL_MCRC_INVALID, 5U, SDL_MCRC_WDTOPLD_MAX+1U, SDL_MCRC_BCTOPLD_MAX+1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_verifyInit(instance, 5U, 255U, SDL_MCRC_BCTOPLD_MAX+1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_init(instance, channel, MCRC_WATCHDOG_PRELOAD, MCRC_BLOCK_PRELOAD);
        if (SDL_MCRC_verifyInit(instance, channel, MCRC_WATCHDOG_PRELOAD, 255U) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_init(instance, channel, MCRC_WATCHDOG_PRELOAD, MCRC_BLOCK_PRELOAD);
        if (SDL_MCRC_verifyInit(instance, channel, 255u, MCRC_BLOCK_PRELOAD) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*---------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of config API*/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_config(instance,channel,patternCount,sectorCount, SDL_MCRC_CTRL2_CH1_MODE_FULLCPU+1U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_config(SDL_MCRC_INVALID,channel,patternCount,sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_config(instance,5U,patternCount,sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_config(instance,channel,(SDL_MCRC_PATTERN_COUNT_MAX+1),sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_config(instance,channel,patternCount,(SDL_MCRC_SECTOR_COUNT_MAX+1), mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*----------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of verify config API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(SDL_MCRC_INVALID,channel,patternCount,sectorCount, SDL_MCRC_OPERATION_MODE_FULLCPU+1U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(SDL_MCRC_INVALID,channel,patternCount,sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode);
        if ((SDL_MCRC_verifyConfig(instance,channel,256U,sectorCount, mode)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode);
        if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,256U, mode)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode);
        if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, SDL_MCRC_OPERATION_MODE_SEMICPU)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode);
        if ((SDL_MCRC_verifyConfig(instance,channel,256U,256U, SDL_MCRC_OPERATION_MODE_SEMICPU)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, SDL_MCRC_CTRL2_CH1_MODE_FULLCPU+1U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(instance,5U,patternCount,sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(instance,channel,(SDL_MCRC_PATTERN_COUNT_MAX+1),sectorCount, mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,(SDL_MCRC_SECTOR_COUNT_MAX+1), mode)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(SDL_MCRC_INVALID,6U,SDL_MCRC_PATTERN_COUNT_MAX+1U,SDL_MCRC_SECTOR_COUNT_MAX, SDL_MCRC_CTRL2_CH1_MODE_FULLCPU+1U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, 6U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-------------------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of channel reset API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_channelReset(SDL_MCRC_INVALID,channel)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_channelReset(instance,5U)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*---------------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of read PSA signature API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(SDL_MCRC_INVALID,channel, &pPSAsign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(instance,5U, &pPSAsign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(instance,channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*---------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of set PSA signature API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_setPSASeedSig(SDL_MCRC_INVALID,channel, &pSeedSign)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_setPSASeedSig(instance,5U, &pSeedSign)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_setPSASeedSig(instance,channel, NULL)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*------------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of read PSA sector signature API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_getPSASectorSig(SDL_MCRC_INVALID,channel,&pSecSign) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_getPSASectorSig(instance,5U,&pSecSign) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_getPSASectorSig(instance,channel,NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*---------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of intrStatus API*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ( SDL_MCRC_getIntrStatus(SDL_MCRC_INVALID, channel, &pIntrstatus)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ( SDL_MCRC_getIntrStatus(instance, 5U, &pIntrstatus)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ( SDL_MCRC_getIntrStatus(instance, channel, NULL)!= SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-----------------------------------------------------------------------------------------*/
    /* Error/Fualt test of EnableIntr API */

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_enableIntr(SDL_MCRC_INVALID, channel, IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_enableIntr(instance, 5U, IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_enableIntr(instance, channel, 255U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*------------------------------------------------------------------------------------------*/
    /* Error/Fualt test of DisableIntr API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_disableIntr(SDL_MCRC_INVALID, channel,IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_disableIntr(instance, 5U,IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_disableIntr(instance, channel,255U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-----------------------------------------------------------------------------------------*/
    /* Error/Fualt test of ClearIntr API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_clearIntr(SDL_MCRC_INVALID, channel,IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_clearIntr(instance, 5U,IntrMask) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MCRC_clearIntr(instance, channel,255U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-------------------------------------------------------------------------*/
    /* Error/Fualt test of SDL_MCRC_isBusy API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_isBusy(SDL_MCRC_INVALID, channel, &pBusyFlag)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_isBusy(instance, 5U, &pBusyFlag)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_isBusy(instance, channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*----------------------------------------------------------------------------------------*/
    /* Error/Fualt test of Get Currrent Sector Number API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurSecNum(SDL_MCRC_INVALID, channel, &pCurSecNum)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurSecNum(instance, 5U, &pCurSecNum)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurSecNum(instance, channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*--------------------------------------------------------------------------------------------*/
    /* Error/Fualt test of Get PSA signature API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(instance, 5U, &pPSAsig)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(SDL_MCRC_INVALID, channel, &pPSAsig)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASig(instance, channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*---------------------------------------------------------------------------------------*/
    /* Error/Fualt test of readStaticreg API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_StaticRegs_t pStaticRegs;
        if ((SDL_MCRC_readStaticReg(SDL_MCRC_INVALID, &pStaticRegs)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_readStaticReg(instance, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*------------------------------------------------------------------------------------*/
    /* Error/Fualt test of GetCurPSASig API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurPSASig(SDL_MCRC_INVALID,channel, &pCurPSASig)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurPSASig(instance,5U, &pCurPSASig)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCurPSASig(instance,channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-----------------------------------------------------------------------------------*/
    /* Error/Fualt test of GetPSASigRegAddr API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASigRegAddr(SDL_MCRC_INVALID,channel, &pMCRCregAddr)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASigRegAddr(instance,5U, &pMCRCregAddr)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getPSASigRegAddr(instance,channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*-----------------------------------------------------------------------------------*/
    /* Error/Fualt test of SDL_MCRC_computeSignCPUmode API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_computeSignCPUmode(SDL_MCRC_INVALID,channel,&pDataConfig, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_computeSignCPUmode(instance,5U, &pDataConfig,&pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_computeSignCPUmode(instance,channel, &pDataConfig, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_computeSignCPUmode(instance,channel, NULL, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_DataConfig_t mcrcData;
        mcrcData.pMCRCData        = NULL_PTR;
        if ((SDL_MCRC_computeSignCPUmode(instance,channel, &mcrcData, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_DataConfig_t mcrcData;
        mcrcData.dataBitSize = SDL_MCRC_DATA_8_BIT;
        mcrcData.pMCRCData        = (uint32_t *)SDL_mcrcTestData;
        if ((SDL_MCRC_computeSignCPUmode(SDL_MCRC_INVALID,channel, &mcrcData, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_DataConfig_t mcrcData;
        mcrcData.dataBitSize = SDL_MCRC_DATA_16_BIT;
        mcrcData.pMCRCData        = (uint32_t *)SDL_mcrcTestData;
        if ((SDL_MCRC_computeSignCPUmode(SDL_MCRC_INVALID,channel, &mcrcData, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_MCRC_DataConfig_t mcrcData;
        mcrcData.dataBitSize = SDL_MCRC_DATA_32_BIT;
        mcrcData.pMCRCData        = (uint32_t *)SDL_mcrcTestData;
        if ((SDL_MCRC_computeSignCPUmode(SDL_MCRC_INVALID,channel, &mcrcData, &pSecSign)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_getCRCRegAddr(instance,channel, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
		SDL_MCRC_SignatureRegAddr_t pCRCRegAddr;
        if ((SDL_MCRC_getCRCRegAddr(SDL_MCRC_INVALID,channel, &pCRCRegAddr)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
		SDL_MCRC_SignatureRegAddr_t pCRCRegAddr;
        if ((SDL_MCRC_getCRCRegAddr(instance, 5u, &pCRCRegAddr)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_configCRCType(SDL_MCRC_INVALID,channel)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_MCRC_configCRCType(instance, 5u)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    return (testStatus);
}
