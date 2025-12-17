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
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

#define MCRC_128KB_BYTES    (uint32_t)(1024 * 128)
#define MCRC_1KB_BYTES      (uint32_t)(1024 * 1)

/* Pre calculated CRC values for profiling datasets */
#define MCRC_128KB_HI       (0x4EB4CABB)
#define MCRC_128KB_LO       (0x432911AF)
#define MCRC_1KB_HI         (0x958B7A02)
#define MCRC_1KB_LO         (0x1871EC9A)

static int32_t mcrcFullProfile(SDL_MCRC_InstType instance, SDL_MCRC_Channel_t channel, SDL_MCRC_DataConfig_t mcrcData,
                                 SDL_MCRC_Signature_t *crc, uint64_t *profTime)
{
    uint64_t profStartTime, profEndTime;

    int32_t testStatus = SDL_PASS;
    SDL_MCRC_channelReset(instance, channel);
    SDL_MCRC_config(instance,channel,mcrcData.size , 1U, SDL_MCRC_OPERATION_MODE_FULLCPU);
    profStartTime = ClockP_getTimeUsec();
    if ((SDL_MCRC_computeSignCPUmode(instance, channel, &mcrcData, crc)) != SDL_PASS)
    {
        testStatus = SDL_EFAIL;
    }
    profEndTime = ClockP_getTimeUsec();
    DebugP_log(" Calculated CRC value is 0x%08x%08x\r\n", crc->regH, crc->regL);
    if (testStatus != SDL_PASS)
    {
        DebugP_log(" mcrcFullProfile API: failure in SDL_MCRC_computeSignCPUmode\r\n");
        /* To set profTime to 0 because of the failure */
        profEndTime = profStartTime;
    }
    *profTime = profEndTime - profStartTime;
    return testStatus;
}
#endif

#if defined(SOC_AM263X)|| defined (SOC_AM64X) || defined (SOC_AM243X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_MCRC_CHANNEL_MAXIMUM 4;
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#define SDL_MCRC_CHANNEL_MAXIMUM 2;
#endif

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AM261X)
 static SDL_MCRC_Config_t SDL_MCRC_Configparam[6] =
 {
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_64BIT,
          SDL_MCRC_DATA_64_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_64BIT,
          SDL_MCRC_DATA_64_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_32BIT,
          SDL_MCRC_DATA_32_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_16BIT,
          SDL_MCRC_DATA_16_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
      {
          SDL_MCRC_CTRL0_CH1_CRC_SEL_64BIT,
          SDL_MCRC_DATALENGTH_64BIT,
          SDL_MCRC_DATA_64_BIT,
          SDL_MCRC_BITSWAP_MSB,
          SDL_MCRC_BYTESWAP_DISABLE
      },
 };
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
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    uint32_t *profMCRCData;
    SDL_MCRC_Signature_t sectSignVal;
    uint64_t profTime;

    /* Profiling test of SDL_MCRC_computeSignCPUmode API */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        DebugP_log("MCRC Profiling Tests: \r\n");
        mcrcData.dataBitSize = SDL_MCRC_DATA_32_BIT;
        mcrcData.pMCRCData = SDL_mcrcProfData;
        profMCRCData = mcrcData.pMCRCData;

        SDL_MCRC_init(instance,channel,0U,0U);
        SDL_MCRC_channelReset(instance,channel);

    #if defined (R5F_CORE)
        for (i = 0; i < MCRC_1KB_BYTES/4; i++)
        {
            profMCRCData[i] = i;
        }
    #else
        for (i = 0; i < MCRC_128KB_BYTES/4; i++)
        {
            profMCRCData[i] = i;
        }

        /* For 128KB data size */
        DebugP_log("Profiling for 128KB dataset\r\n");
        mcrcData.size = MCRC_128KB_BYTES;
        testStatus = mcrcFullProfile(instance, channel, mcrcData, &sectSignVal, &profTime);
        if (testStatus == SDL_PASS)
        {
            if (sectSignVal.regH != MCRC_128KB_HI || sectSignVal.regL != MCRC_128KB_LO)
            {
                DebugP_log(" Error: MCRC value does not match for 128KB \r\n");
                testStatus = SDL_EFAIL;
            }
            else
            {
                DebugP_log(" Calculated and Expected CRC values match for 128KB \r\n");
            }
        }
        if (testStatus == SDL_PASS)
        {
            DebugP_log(" MCRC Profiling result: 128KB ~ %dus \r\n", profTime);
        }
        else
        {
            DebugP_log(" Error in MCRC Profiling run for 128KB \r\n");
        }
        DebugP_log("\r\n");
    #endif
        /* For 1KB data size */
        DebugP_log("Profiling for 1KB dataset\r\n");
        mcrcData.size = MCRC_1KB_BYTES;
        testStatus = mcrcFullProfile(instance, channel, mcrcData, &sectSignVal, &profTime);
        if (testStatus == SDL_PASS)
        {
            if (sectSignVal.regH != MCRC_1KB_HI || sectSignVal.regL != MCRC_1KB_LO)
            {
                DebugP_log(" Error: MCRC value does not match for 1KB \r\n");
                testStatus = SDL_EFAIL;
            }
            else
            {
                DebugP_log(" Calculated and Expected CRC values match for 1KB \r\n");
            }
        }
        if (testStatus == SDL_PASS)
        {
            DebugP_log(" MCRC Profiling result: 1KB ~ %dus \r\n", profTime);
        }
        else
        {
            DebugP_log(" Error in MCRC Profiling run for 1KB \r\n");
        }
        DebugP_log("\r\n");
    }
#endif

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
        #if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM273X) ||defined(SOC_AWR294X) || defined (SOC_AM261X)
        for (bit_size=SDL_MCRC_DATA_8_BIT; bit_size<= SDL_MCRC_DATA_64_BIT; bit_size++)
        #else
        for (bit_size=SDL_MCRC_DATA_8_BIT; bit_size<= SDL_MCRC_DATA_32_BIT; bit_size++)
        #endif
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
        mcrcData.dataBitSize     =(SDL_MCRC_DataBitSize) (SDL_MCRC_DATA_32_BIT + 3);
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
                    #if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X) || defined (SOC_AM273X) || defined (SOC_AWR294x)
                    SDL_MCRC_Configparam[0].type = SDL_MCRC_TYPE_64BIT;
                    if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, mode, &SDL_MCRC_Configparam[0])) != SDL_PASS)
                    #else
                    if ((SDL_MCRC_verifyConfig(instance,channel,patternCount,sectorCount, mode)) != SDL_PASS)
                    #endif
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
            #if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AM261X)
            if ((SDL_MCRC_addConfig(instance,channel,&SDL_MCRC_Configparam[channel-1U]) != SDL_PASS))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            SDL_MCRC_Configparam[channel-1U].type = SDL_MCRC_TYPE_16BIT;
            if ((SDL_MCRC_addConfig(instance,channel,&SDL_MCRC_Configparam[channel-1U]) != SDL_PASS))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            SDL_MCRC_Configparam[channel-1U].type = SDL_MCRC_TYPE_32BIT;
            if ((SDL_MCRC_addConfig(instance,channel,&SDL_MCRC_Configparam[channel-1U]) != SDL_PASS))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            SDL_MCRC_Configparam[channel-1U].type = SDL_MCRC_TYPE_E2EPROFILE;
            if ((SDL_MCRC_addConfig(instance,channel,&SDL_MCRC_Configparam[channel-1U]) != SDL_PASS))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            SDL_MCRC_Configparam[channel-1U].type = SDL_MCRC_TYPE_VDA_CAN_SAEJ1850;
            if ((SDL_MCRC_addConfig(instance,channel,&SDL_MCRC_Configparam[channel-1U]) != SDL_PASS))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
            if (testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("SDL_mcrc_api_pos_Test: failure on line no. %d \n", __LINE__);
                return (testStatus);
            }
            #endif
        }

    }



    return (testStatus);
}
