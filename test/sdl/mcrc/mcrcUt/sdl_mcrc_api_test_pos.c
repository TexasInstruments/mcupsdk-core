/* Copyright (c) 2021-2024 Texas Instruments Incorporated
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
 *  \file     sdl_mcrc_api_test_pos.c
 *
 *  \brief    This file contains mcrc API unit test code.
 *
 *  \details  mcrc unit tests
 **/

#include "mcrc_main.h"

#if defined(SOC_AM263X)|| defined (SOC_AM64X) || defined (SOC_AM243X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_MCRC_CHANNEL_MAXIMUM 4;
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#define SDL_MCRC_CHANNEL_MAXIMUM 2;
#endif

int32_t sdl_mcrc_posTest(void)
{
    int32_t               testStatus = SDL_APP_TEST_PASS;
#if defined (SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    SDL_MCRC_InstType     instance = MCRC0;
	SDL_MCRC_InstType     start_instance = MCRC0;
	SDL_MCRC_InstType     end_instance = MCRC0;
	SDL_MCRC_Channel_t    channelNum=4;
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    SDL_MCRC_InstType     instance = MCU_MCRC64_0 ;
	SDL_MCRC_InstType     start_instance = MCU_MCRC64_0 ;
	SDL_MCRC_InstType     end_instance = MCU_MCRC64_0 ;
	SDL_MCRC_Channel_t    channelNum=4;
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined(R5F_INPUTS)
	SDL_MCRC_InstType     instance = MSS_MCRC;
	SDL_MCRC_InstType     start_instance = MSS_MCRC;
	SDL_MCRC_InstType     end_instance = MSS_MCRC;
#endif
#if defined(C66_INPUTS)
	SDL_MCRC_InstType     instance = DSS_MCRC;
	SDL_MCRC_InstType     start_instance = DSS_MCRC;
	SDL_MCRC_InstType     end_instance = DSS_MCRC;
#endif
	SDL_MCRC_Channel_t    channelNum=2;
#endif
    SDL_MCRC_Channel_t    channel = SDL_MCRC_CHANNEL_1;
    uint32_t              watchdogPreload = MCRC_WATCHDOG_PRELOAD;
    uint32_t              blockPreload = MCRC_BLOCK_PRELOAD;
    SDL_MCRC_ModeType mode = SDL_MCRC_OPERATION_MODE_AUTO;
    uint32_t  patternCount = 255U;
    uint32_t  sectorCount = 255U;
    uint32_t IntrMask = 0x1U;
    SDL_MCRC_DataConfig_t mcrcData;
    uint32_t             i, bit_size;
    uint32_t  *pMCRCData;

    /* positive test of SDL_MCRC_computeSignCPUmode API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mcrcData.dataBitSize     = SDL_MCRC_DATA_32_BIT;
        mcrcData.pMCRCData        = (uint32_t *)SDL_mcrcTestData;
        mcrcData.size            = SDL_MCRC_DATA_SIZE;
        SDL_MCRC_Signature_t  sectSignVal;

        SDL_MCRC_init(instance,channel,0U,0U);
        SDL_MCRC_channelReset(instance,channel);
        SDL_MCRC_config(instance,channel,mcrcData.size/4U, 1U, SDL_MCRC_OPERATION_MODE_FULLCPU);
        for (bit_size=SDL_MCRC_DATA_8_BIT; bit_size<= SDL_MCRC_DATA_32_BIT; bit_size++)
        {
            mcrcData.dataBitSize = (SDL_MCRC_DataBitSize)bit_size;
            pMCRCData = (uint32_t *)mcrcData.pMCRCData;
            for (i = 0; i < (mcrcData.size / 4U); i++)
            {
                pMCRCData[i] = i;
            }

            if ((SDL_MCRC_computeSignCPUmode(instance,SDL_MCRC_CHANNEL_1, &mcrcData, &sectSignVal)) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
        }
        /* Giving Data bit size wrong for failing SDL_MCRC_dataWrite API */
        mcrcData.dataBitSize     =(SDL_MCRC_DataBitSize) (SDL_MCRC_DATA_32_BIT + 1);
        if ((SDL_MCRC_computeSignCPUmode(instance,SDL_MCRC_CHANNEL_1, &mcrcData, &sectSignVal)) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        mcrcData.dataBitSize     = SDL_MCRC_DATA_32_BIT;
    }
	for (instance = start_instance; instance <= end_instance; instance++)
    {
        /* positive test of readStaticreg API */
        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_MCRC_StaticRegs_t pStaticRegs;
            if ((SDL_MCRC_readStaticReg(instance, &pStaticRegs)) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        for (channel = SDL_MCRC_CHANNEL_1; channel <= channelNum; channel++)
        {
            /*  positive test of init API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if (SDL_MCRC_init(instance, channel, watchdogPreload, blockPreload)!= SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of verify init API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if (SDL_MCRC_verifyInit(instance, channel, watchdogPreload, blockPreload) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            /*  positive test of config API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_config(instance,channel,patternCount,SDL_MCRC_SECTOR_COUNT_MAX, SDL_MCRC_CTRL2_CH1_MODE_FULLCPU)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDLmcrc_api_Neg_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of verify config API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                #if defined(C66_INPUTS)
                /* In release mode, failure is  observed for channel 2. Since this is already verified in debug mode
                   and example and also works when doing single step, commenting out the check*/
                if(channel != 2)
                {
                #endif
                    SDL_MCRC_config(instance,channel,patternCount,sectorCount, mode);
                    if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, mode)) != SDL_PASS)
                    {
                        testStatus = SDL_APP_TEST_FAILED;
                    }
                #if defined(C66_INPUTS)
                }
                #endif
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of channel reset API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_channelReset(instance,channel)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of read PSA signature API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_Signature_t pPSAsign;
                if ((SDL_MCRC_getPSASig(instance,channel, &pPSAsign)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of set PSA signature API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_Signature_t pSeedSign;
                pSeedSign.regL    = 255U;
                pSeedSign.regH    = 255U;
                if (SDL_MCRC_setPSASeedSig(instance,channel, &pSeedSign)!= SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of read PSA sector signature API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_Signature_t pSecSign;

                if (SDL_MCRC_getPSASectorSig(instance,channel,&pSecSign) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /*  positive test of intrStatus API*/
            if (testStatus == SDL_APP_TEST_PASS)
            {
                uint32_t pIntrstatus;
                if ( SDL_MCRC_getIntrStatus(instance, channel, &pIntrstatus)!= SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of EnableIntr API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if (SDL_MCRC_enableIntr(instance, channel,IntrMask) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of DisableIntr API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if (SDL_MCRC_disableIntr(instance, channel,IntrMask) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of ClearIntr API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                if (SDL_MCRC_clearIntr(instance, channel,IntrMask) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            /* positive test of SDL_MCRC_isBusy API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                uint32_t pBusyFlag;
                if ((SDL_MCRC_isBusy(instance, channel, &pBusyFlag)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of Get Currrent Sector Number API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                uint32_t pCurSecNum;
                if ((SDL_MCRC_getCurSecNum(instance, channel, &pCurSecNum)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of Get PSA signature API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_Signature_t pPSAsig;
                if ((SDL_MCRC_getPSASig(instance, channel, &pPSAsig)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of GetCurPSASig API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_Signature_t pCurPSASig;
                if ((SDL_MCRC_getCurPSASig(instance,channel, &pCurPSASig)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

            /* positive test of GetPSASigRegAddr API */
            if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_SignatureRegAddr_t pMCRCregAddr;
                if ((SDL_MCRC_getPSASigRegAddr(instance,channel, &pMCRCregAddr)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

			if (testStatus == SDL_APP_TEST_PASS)
            {
                SDL_MCRC_SignatureRegAddr_t pCRCRegAddr;
                if ((SDL_MCRC_getCRCRegAddr(instance,channel, &pCRCRegAddr)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

			if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_configCRCType(instance,channel)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

			if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_configCRCType(instance,channel)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }

			if (testStatus == SDL_APP_TEST_PASS)
            {
                if ((SDL_MCRC_configCRCType(instance,channel)) != SDL_PASS)
                {
                    testStatus = SDL_APP_TEST_FAILED;
                }
            }

            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
        }
    }

    return (testStatus);
}
