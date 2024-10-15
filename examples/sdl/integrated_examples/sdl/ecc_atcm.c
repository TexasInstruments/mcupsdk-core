/*
 *  Copyright (C) Texas Instruments Incorporated 2024
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
 *  \file     ecc_atcm.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Correcting Code (ECC) Module application.
 *
 *  \details  ECC Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include "sdlexample.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS                (1u)
#define SDL_EXAMPLE_ECC_ATCM_RAM_ADDR                    (0x00000510u) /* R5F ATCM0 RAM address */
#define SDL_EXAMPLE_ECC_ATCM_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_ATCM_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID

#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS				(0x50D18094u)
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW			(0x50D18098u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS 				(0x50D18084u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW			(0x50D18088u)

#define SDL_CLEAR_STATUS									(0x01u)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile static bool esmEccError = false;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ecc_atcm_clearESM(void)
{
    /* Clear DED MSS_CTRL register*/
    SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
    /* Clear DED RAW MSS_CTRL register*/
    SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);

    /* Clear SEC MSS_CTRL register*/
    SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
    /* Clear SEC RAW MSS_CTRL register*/
    SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
    
    esmEccError = true;
}

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_ATCM_RAM_ADDR);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_ATCM_AGGR,
                                 SDL_EXAMPLE_ECC_ATCM_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        testLocationValue++;
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_ATCM_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x30002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_ATCM_AGGR,
                                 SDL_EXAMPLE_ECC_ATCM_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        testLocationValue++;
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_ATCMsdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t ECC_ATCMsdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;

    if (retVal == 0)
    {
        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest();
        if (result == SDL_PASS)
        {
            /* waiting for ESM interrupt */
            do
            {
                timeOutCnt += 1;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmEccError == false);
        }
        if(result == SDL_PASS){
            esmEccError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            /* UC-1 Low priority R5F interrupt */
        }
    }
    if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest();
        if (result == SDL_PASS)
        {
            /* waiting for ESM interrupt */
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmEccError == false);
        }
        if(result == SDL_PASS){
            esmEccError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            /* UC-2 High priority R5F interrupt */
        }
    }

    return retVal;
}

/* ECC Function module test */
int32_t ECC_ATCM_test(void)
{
    int32_t testResult = 0;

    /*Enabling the ECC module*/
    SDL_ECC_UTILS_enableECCATCM();

    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    /*Execute ECC sdl function test*/
    testResult = ECC_ATCMsdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
