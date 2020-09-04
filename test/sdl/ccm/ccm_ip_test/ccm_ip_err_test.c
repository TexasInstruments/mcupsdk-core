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
 *  \file     ccm_ip_err_test.c
 *
 *  \brief    This file contains CCM IP Error module test code for R5 core.
 *
 *  \details  CCM IP Error module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/sdl_types.h>
#include "ccm_test_main.h"
#include <sdl/r5/v0/sdl_mcu_armss_ccmr5.h>

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

static int32_t CCM_errTest(void)
{
    int32_t  testResult = SDL_PASS;
    SDL_ErrType_t sdlResult;
	uint32_t    valToBeRead;
	int32_t     metaInfo;
    uint32_t      cmpError;
    SDL_McuArmssCcmR5OpModeKey    opModeKey;

    sdlResult = SDL_armR5ReadCCMRegister((uintptr_t) NULL, (SDL_McuArmssCcmR5RegId)0U, &valToBeRead, &metaInfo);
    if (sdlResult == SDL_PASS)
    {
        DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
        testResult = -1;
    }

    if (testResult == 0)
    {
        sdlResult = SDL_armR5ReadCCMRegister(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5RegId)0U, NULL, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

	if(testResult == 0)
	{
		sdlResult = SDL_armR5ReadCCMRegister(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5RegId)7U, &valToBeRead, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
		sdlResult = SDL_armR5ConfigureCCMRegister((uintptr_t) NULL, (SDL_McuArmssCcmR5RegId)0U, valToBeRead, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
		sdlResult = SDL_armR5CCMSetOperationModeKey((uintptr_t) NULL, (SDL_McuArmssCcmR5ModuleId)0U, (SDL_McuArmssCcmR5OpModeKey)0U, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)255u, (SDL_McuArmssCcmR5OpModeKey)0U, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)0u, (SDL_McuArmssCcmR5OpModeKey)255U, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
		sdlResult = SDL_armR5CCMGetCompareError((uintptr_t) NULL, (SDL_McuArmssCcmR5ModuleId)0U, &cmpError, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMGetCompareError(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)255u, &cmpError, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMGetCompareError(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)0U, NULL, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
		sdlResult = SDL_armR5CCMGetOperationModeKey((uintptr_t) NULL, (SDL_McuArmssCcmR5ModuleId)0U, &opModeKey, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)255u, &opModeKey, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)0U, NULL, &metaInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
		sdlResult = SDL_armR5CCMClearCompareError((uintptr_t) NULL, (SDL_McuArmssCcmR5ModuleId)0U, &metaInfo);
		if (sdlResult == SDL_PASS)
		{
			DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}

    if (testResult == 0)
    {
        sdlResult = SDL_armR5CCMClearCompareError(SDL_CCM_baseAddress[0], (SDL_McuArmssCcmR5ModuleId)255u, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    return (testResult);
}

/* CCM Error module test */
int32_t CCM_ipErrTest(void)
{
    int32_t    testResult;

    testResult = CCM_errTest();

    return (testResult);
}

/* Nothing past this point */
