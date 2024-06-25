/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     ecc_test_func.c
 *
 *  \brief    This file contains ECC Functional test code.
 *
 *  \details  ECC Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
#include <sdl/include/awr294x/sdlr_soc_ecc_aggr.h>
#include "ecc_test_main.h"
#include <sdl/ecc/sdl_ecc_core.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t ECC_funcAPITest(void)
{
    int32_t testStatus = SDL_APP_TEST_PASS;

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_configECCRam(20U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_pollErrorEvent(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID, \
                                    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_pollErrorEvent(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID, \
                                    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_pollErrorEvent(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID, \
                                    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_pollErrorEvent(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID, \
                                    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_pollErrorEvent(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID, \
                                    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    return (testStatus);
}

/* ECC Functional test */
int32_t ECC_r5_funcTest(void)
{
    int32_t testResult = 0;

    testResult = ECC_funcAPITest();

    return (testResult);
}
/* Nothing past this point */
