/*
 *  Copyright (C) Texas Instruments Incorporated 2022-2023
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
 *  \file sdl_mcrcFullCPU.c
 *
 *  \brief Common across test-cases using MCRC Full-CPU mode.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "mcrc_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define MAX_LOOPCOUNT                  (20U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/** \brief Defines the various MCRC test cases. */
static SDL_MCRC_ConfigParams_t testParams[2] =
{
    {
#if defined(SOC_AM64X) || defined (SOC_AM243X)
		MCU_MCRC64_0,
#endif

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
		MCRC0,
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
		MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_2,
        (uint32_t) SDL_MCRC_OPERATION_MODE_FULLCPU,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x3668f7eaU,
        0xaef33083U,
        MCRC_BUF_MAX_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
   },
   {
#if defined(SOC_AM64X)  || defined (SOC_AM243X)
		MCU_MCRC64_0,
#endif
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
		MCRC0,
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
		MCRC_INSTANCE,
#endif
        (uint32_t) SDL_MCRC_CHANNEL_1,
        (uint32_t) SDL_MCRC_OPERATION_MODE_FULLCPU,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x3668f7eaU,
        0xaef33083U,
        MCRC_BUF_MAX_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
};

int32_t sdl_mcrcFullCPU_main(void)
{
    int32_t           result;
    int32_t              retVal=0;
    SDL_MCRC_DataConfig_t mcrcData;
    uint32_t             i;
    uint32_t  *pMCRCData;
    uint32_t testCase;

    for(testCase=0; testCase<=1; testCase++)
    {
        DebugP_log("\n MCRC FULL CPU mode test: starting");

        mcrcData.pMCRCData       = (uint32_t *)testParams[testCase].sourceMemory;
        mcrcData.size           = testParams[testCase].dataSize;
        mcrcData.dataBitSize     = SDL_MCRC_DATA_32_BIT;
        SDL_MCRC_Signature_t  sectSignVal;
        SDL_MCRC_StaticRegs_t pStaticRegs;

        SDL_MCRC_readStaticReg(testParams[testCase].instance, &pStaticRegs);
        DebugP_log("\n Static PCOUNT is : %d", pStaticRegs.channelRegs[0].PCOUNT);

        result = SDL_MCRC_init(testParams[testCase].instance,testParams[testCase].mcrcChannelNumber,
                               testParams[testCase].mcrcWatchdogPreload,testParams[testCase].mcrcBlockPreload);
        if (result == SDL_PASS)
        {
            result = SDL_MCRC_channelReset(testParams[testCase].instance,testParams[testCase].mcrcChannelNumber);
            SDL_MCRC_config(testParams[testCase].instance,testParams[testCase].mcrcChannelNumber,testParams[testCase].mcrcPatternCount,
                        testParams[testCase].mcrcSectorCount, testParams[testCase].mcrcMode);
            result = SDL_MCRC_verifyConfig(testParams[testCase].instance,testParams[testCase].mcrcChannelNumber,testParams[testCase].mcrcPatternCount,
                        testParams[testCase].mcrcSectorCount, testParams[testCase].mcrcMode);
        }
        if (result == SDL_PASS)
        {
            DebugP_log("\n Configuration verified");
        }

        pMCRCData = (uint32_t *)mcrcData.pMCRCData;
        for (i = 0; i < (mcrcData.size / 4U); i++)
        {
            pMCRCData[i] = i;
        }

        result = SDL_MCRC_computeSignCPUmode(testParams[testCase].instance,
                                testParams[testCase].mcrcChannelNumber,
                                &mcrcData, &sectSignVal);

        SDL_MCRC_readStaticReg(testParams[testCase].instance, &pStaticRegs);
        DebugP_log("\n Static PCOUNT is : %d", pStaticRegs.channelRegs[0].PCOUNT);
        if (result == SDL_PASS)
        {
            /*
             * Check if the generated MCRC signature value
             * matches with the reference signaure vaule
             */
            if ((sectSignVal.regH == testParams[testCase].mcrcSignHigh) &&
                (sectSignVal.regL == testParams[testCase].mcrcSignLow))
            {
                result = SDL_PASS;
            }
            else
            {
                DebugP_log("\n regH is 0x%x regL is 0x%x\n", sectSignVal.regH, sectSignVal.regL);
                result = SDL_EFAIL;
            }
        }

        if (result != SDL_PASS)
        {
#if defined (SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
            if (testParams[testCase].instance == MCRC0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCRC0 \n\n");
            }
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
            if (testParams[testCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCU_NAVSS \n\n");
            }
#endif
#if defined (SOC_AM243X)
            if (testParams[testCase].instance ==  MCU_MCRC64_0  )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCU_MCRC64_0 \n\n");
            }
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
            if (testParams[testCase].instance == MCRC_INSTANCE )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed\n\n");
            }
#endif
        }
        else
        {
#if defined (SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
            if (testParams[testCase].instance == MCRC0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCRC0 \n\n ");
            }
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
            if (testParams[testCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCU_NAVSS \n\n ");
            }
#endif

#if defined (SOC_AM243X)
            if (testParams[testCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCU_MCRC64_0 \n\n ");
            }
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
            if (testParams[testCase].instance == MCRC_INSTANCE )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfull\n\n");
            }
#endif
            retVal = SDL_PASS;
        }

        if (retVal == SDL_EFAIL)
        {
            break;
        }
    }

    return retVal;
}

