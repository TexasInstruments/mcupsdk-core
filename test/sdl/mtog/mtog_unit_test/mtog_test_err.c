/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     mtog_test_err.c
 *
 *  \brief    This file contains MTOG error tests.
 *
 *  \details  MTOG Error tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_mtog.h>
#include <kernel/dpl/DebugP.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t MTOG_errNegativeTest(uint32_t instanceIndex)
{
    int32_t retVal, testResult = 0;
    SDL_MTOG_Regs *regs;
	uint32_t baseAddr;
    regs  = (SDL_MTOG_Regs *)SDL_MTOG_getBaseaddr(instanceIndex, &baseAddr);
	SDL_MTOG_config config;
    SDL_MTOG_staticRegs  staticRegs;
	config.timeOut = SDL_MTOG_VAL_2M;
	
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_setTimeoutVal(NULL, SDL_MTOG_VAL_1K);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_setTimeoutVal error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        /* To get EFAIL, enabled MTOG_EN */
        SDL_MTOG_start(instanceIndex);
        retVal = SDL_MTOG_setTimeoutVal(regs, SDL_MTOG_VAL_1K);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_setTimeoutVal error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        /* To get EFAIL, enabled MTOG_EN */
        SDL_MTOG_start(instanceIndex);
        retVal = SDL_MTOG_init(instanceIndex, &config);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_setTimeoutVal error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
        SDL_MTOG_stop(instanceIndex);
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_setTimeoutVal(regs, SDL_MTOG_VAL_4M_MINUS_1 + 1U);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_setTimeoutVal error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_init(SDL_INSTANCE_MTOG_MAX + 1U, &config);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_init error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_init(instanceIndex, NULL);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_init error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_verifyConfig(SDL_INSTANCE_MTOG_MAX + 1U, &config);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_verifyConfig error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_verifyConfig(instanceIndex, NULL);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_verifyConfig error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
		/* To get EFAIL, different config value passed */
        retVal = SDL_MTOG_verifyConfig(instanceIndex, &config);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_verifyConfig error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_start(SDL_INSTANCE_MTOG_MAX + 1U);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_start error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_stop(SDL_INSTANCE_MTOG_MAX + 1U);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_stop error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_forceTimeout(SDL_INSTANCE_MTOG_MAX + 1U);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_forceTimeout error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
		/* To get EFAIL, stopped MTOG */
		SDL_MTOG_stop(instanceIndex);
        retVal = SDL_MTOG_forceTimeout(instanceIndex);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_forceTimeout error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_reset(SDL_INSTANCE_MTOG_MAX + 1U);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_reset error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_getStaticRegisters(SDL_INSTANCE_MTOG_MAX + 1U, &staticRegs);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_getStaticRegisters error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        retVal = SDL_MTOG_getStaticRegisters(instanceIndex, NULL);
        if (retVal == SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_getStaticRegisters error test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    return (testResult);
}

/* MTOG Error module test */
int32_t MTOG_errTest(void)
{
    int32_t testResult;

    testResult = MTOG_errNegativeTest(SDL_INSTANCE_MCU_MTOG0);

    return (testResult);
}
/* Nothing past this point */