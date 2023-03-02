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
 *  \file     mtog_test_api.c
 *
 *  \brief    This file contains MTOG API test code. .
 *
 *  \details  MTOG API tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_mtog.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/mtog/soc/am64x/sdl_soc_mtog.h>

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

/* Run all APIs not exercised by functional test */
static int32_t MTOG_apiTestLocal(uint32_t instanceIndex)
{
    int32_t testResult = SDL_PASS;
    SDL_MTOG_Regs *regs;
	uint32_t baseAddr;
    regs  = (SDL_MTOG_Regs *)SDL_MTOG_getBaseaddr(instanceIndex, &baseAddr);
    SDL_MTOG_config config;
    SDL_MTOG_staticRegs  staticRegs;
	config.timeOut = SDL_MTOG_VAL_1K;

    if (testResult == SDL_PASS)
    {
        if (SDL_MTOG_setTimeoutVal(regs, SDL_MTOG_VAL_1K)!= SDL_EBADARGS)
        {
            DebugP_log("\n  SDL_MTOG_setTimeoutVal API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_init(instanceIndex, &config);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_init API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_verifyConfig(instanceIndex, &config);
        if (testResult != SDL_PASS)
        {
           DebugP_log("\n  SDL_MTOG_verifyConfig API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_start(instanceIndex);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_start API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	if (testResult == SDL_PASS)
    {
	  SDL_MTOG_start(instanceIndex);
      testResult = SDL_MTOG_forceTimeout(instanceIndex);
      if (testResult != SDL_PASS)
      {
          DebugP_log("\n  SDL_MTOG_forceTimeout API test failed on line no: %d \n", __LINE__);
          testResult = -1;
      }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_stop(instanceIndex);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_stop API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_getStaticRegisters(instanceIndex, &staticRegs);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_MTOG_getStaticRegisters API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	if (testResult == SDL_PASS)
    {
        testResult = SDL_MTOG_getBaseaddr(instanceIndex, NULL);
        if (testResult != SDL_PASS)
        {
            testResult = 0;
        }
		else{
			DebugP_log("\n  SDL_MTOG_getStaticRegisters API test failed on line no: %d \r\n", __LINE__);
			testResult = -1;
		}
		
    }
	
    return (testResult);	
}


/* MTOG API module test */
int32_t MTOG_apiTest(void)
{
    int32_t testResult;
	
    testResult = MTOG_apiTestLocal(SDL_INSTANCE_MCU_MTOG0);
	
    return (testResult);
}
/* Nothing past this point */