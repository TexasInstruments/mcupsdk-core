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
 *  \file     ccm_sdl_err_test.c
 *
 *  \brief    This file contains CCM Error module test code for R5 core.
 *
 *  \details  CCM Error module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/sdl_types.h>
#include <ccm_test_main.h>
#include <sdl/r5/v0/sdl_ip_ccm.h>
#include <sdl/r5/v0/sdl_mcu_armss_ccmr5.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define INSTANCE 		SDL_R5SS0_CCM
#define ESM_INSTANCE 	SDL_ESM_INST_MAIN_ESM0
#elif defined(SOC_AM273X) || defined(SOC_AWR294X)
#define INSTANCE 		SDL_MSS_CCMR
#define ESM_INSTANCE 	SDL_ESM_INST_MSS_ESM
#endif
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
	SDL_CCM_staticRegs staticRegs;
    SDL_CCM_MonitorType monitorType;
	uint32_t instanceId = 0U;
    sdlResult = SDL_CCM_init(SDL_CCM_MAX_INSTANCE, 0U);
    if (sdlResult == SDL_PASS)
    {
        DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
        testResult = -1;
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_verifyConfig(SDL_CCM_MAX_INSTANCE);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
	if(testResult == 0)
	{
		/* To get EFAIL, set differnt TEST MODE */
	    SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instanceId],   \
                                                   SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID, \
                                                   SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE, \
    	    									   NULL );
    	sdlResult = SDL_CCM_verifyConfig(instanceId);
    	if (sdlResult != SDL_EFAIL)
    	{
    		DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
    }

   if(testResult == 0)
	{
		/* To get EFAIL, set differnt TEST MODE */
	    SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instanceId],   \
                                                   SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID, \
                                                   SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE, \
    	    									   NULL );
    	sdlResult = SDL_CCM_verifyConfig(instanceId);
    	if (sdlResult != SDL_EFAIL)
    	{
    		DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}

	if(testResult == 0)
	{
		/* To get EFAIL, set differnt TEST MODE */
	    SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instanceId],   \
                                                   SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID, \
                                                   SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE, \
    	    									   NULL );
    	sdlResult = SDL_CCM_verifyConfig(instanceId);
    	if (sdlResult != SDL_EFAIL)
    	{
    		DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
    	sdlResult = SDL_CCM_getStaticRegisters(instanceId, &staticRegs);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_getStaticRegisters(SDL_CCM_MAX_INSTANCE, &staticRegs);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_getStaticRegisters(INSTANCE, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

	if (testResult == 0)
    {
        sdlResult = SDL_CCM_clearError(SDL_CCM_MAX_INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_clearError(INSTANCE, SDL_CCM_MONITOR_TYPE_INVALID);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_CCM_selfTest(SDL_CCM_MAX_INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK, \
		                             SDL_CCM_SELFTEST_TYPE_NORMAL, 0x0FF, 1000000U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_INVALID, \
		                             SDL_CCM_SELFTEST_TYPE_NORMAL, 0x0FF, 1000000U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK, \
		                             SDL_CCM_SELFTEST_TYPE_INVALID, 0x0FF, 1000000U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }


    if (testResult == 0)
    {
        sdlResult = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_VIM, \
		                             SDL_CCM_SELFTEST_TYPE_INVALID, 0x0FF, 1000000U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK, \
		                             SDL_CCM_SELFTEST_TYPE_NORMAL, 0x0, 1000000U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_injectError(SDL_CCM_MAX_INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_injectError(INSTANCE, SDL_CCM_MONITOR_TYPE_INVALID);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_getErrorType(SDL_CCM_MAX_INSTANCE, ESM_INSTANCE, &monitorType);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_getErrorType(INSTANCE, 0x00, &monitorType);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlResult = SDL_CCM_getErrorType(INSTANCE, ESM_INSTANCE, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("sdlCcm_negTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    return (testResult);
}

/* CCM Error module test */
int32_t CCM_sdlErrTest(void)
{
    int32_t    testResult;

    testResult = CCM_errTest();

    return (testResult);
}

/* Nothing past this point */
