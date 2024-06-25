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
#include "ccm_ip_test_main.h"
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
	SDL_vimRegs *pRegs;
#if defined(SOC_AM273X)||defined(SOC_AWR294X)
	/* initialize the address */
	pRegs        = (SDL_vimRegs *)(uintptr_t)SDL_MSS_VIM_R5A_U_BASE;
#endif
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
	/* initialize the address */
	pRegs        = (SDL_vimRegs *)(uintptr_t)SDL_VIM_U_BASE;
#endif
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
	if (testResult == 0)
    {
		SDL_vimStaticRegs staticRegs;
        sdlResult = SDL_VIM_getStaticRegs(NULL, &staticRegs);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_getStaticRegs(pRegs, NULL);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
	if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(NULL, 0xFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFEU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFCU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(NULL, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFU, 0xFFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFU, 0xFU, 0x2U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x2U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x1U, 0xFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFFFFFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFFFFFEU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x1U, 0x00FFFFFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(pRegs, 0x200U, 0xFU, 0x1U, 0x1U, 0x00FFFFF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_cfgIntr(NULL, 0xFFU, 0xFU, 0x1U, 0x1U, 0x00FFFFF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
	if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(NULL, 0xFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(NULL, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFEU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xFFFCU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFFFU, 0xFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFU, 0xFFU, 0x1U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFU, 0xFU, 0x2U, 0x1U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x2U, 0xF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x1U, 0xFFU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 11U, 0xFU, 0x1U, 0x1U, 1U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 11U, 0xFU, 0x0U, 0x0U, 1U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 11U, 0x0U, 0x0U, 0x1U, 1U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 11U, 0xFU, 0x0U, 0x1U, 0x00FFFFF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0xFFU, 0xFU, 0x1U, 0x1U, 0xFFFFFFFDU);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_verifyCfgIntr(pRegs, 0x200U, 0xFU, 0x1U, 0x1U, 0x00FFFFF0U);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
        else{
            testResult = 0;
        }
    }
	if (testResult == 0)
    {
		SDL_vimStaticRegs staticRegs;
        sdlResult = SDL_VIM_getStaticRegs(NULL, &staticRegs);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_VIM_getStaticRegs(pRegs, NULL);

        if (sdlResult == SDL_PASS)
        {
            testResult = -1;
            DebugP_log("\n  VIM Error test failed on line no: %d \n", __LINE__);
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
