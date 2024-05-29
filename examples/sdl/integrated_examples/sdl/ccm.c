/*
 *   Copyright (c) Texas Instruments Incorporated 2024
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
 *  \file     ccm.c
 *
 *  \brief    This file contains R5F-CPU Safety Example showing usage of R5F Lockstep as a safety mechanism to detect errors in R5F execution. Example shows how to configure and use all 4 modes. Example also show how to configure PMU, MPU and illegal instruction trapping.
 *
 *  \details  R5F-CPU running in lockstep mode
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ccm.h>
#include <sdl/r5/v0/sdl_ip_ccm.h>
#include "sdlexample.h"


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define CCM_ERROR_INDEX   0U

/* ========================================================================== */
/*                            Internal Function Declaration                    */
/* ========================================================================== */
/*********************************************************************
 * @fn      SDL_TEST_CCMSelfTest
 *
 * @brief   Execute CCM Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t cpu_example_app(uint32_t instance);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
int32_t loop = 0u;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t SDL_TEST_CCMSelfTest(uint32_t ccmInstance)
{
    int32_t result;

    result = SDL_CCM_selfTest(ccmInstance,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    return result;
}


/*********************************************************************
 * @fn      SDL_TEST_CCMSelfTestErrorForce
 *
 * @brief   Execute CCM Self test with error force
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMSelfTestErrorForce(uint32_t InstanceCCM)
{
    int32_t result;
    result = SDL_CCM_selfTest(InstanceCCM,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    return result;
}

/*********************************************************************
 * @fn      SDL_TEST_CCMInjectError
 *
 * @brief   Execute CCM Inject Error test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMInjectError(uint32_t ccmInstanceID)
{
    int32_t result;

    result = SDL_CCM_injectError(ccmInstanceID, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);

    return result;
}

static int32_t CCM_runTest(uint32_t instanceId)
{
    int32_t       testResult = 0;

	testResult = cpu_example_app(instanceId);
    return (testResult);
}


int32_t CCM_Test_init (int32_t instNum)
{
    int32_t result = SDL_PASS;

    /* Initialize CCM */
    result = SDL_CCM_init(instNum, CCM_ERROR_INDEX);

    if (result == SDL_PASS)
    {
        /* Verify CCM Configuration */
        result = SDL_CCM_verifyConfig(instNum);
    }

    return result;
}

/* CCM Functional test */
int32_t CCM_test(void *args)
{
    int32_t    testResult = SDL_PASS;

	for(loop = 0; loop < SDL_CCM_MAX_INSTANCE; loop++)
	{
		/* initialize CCM and verify configuration */
		testResult = CCM_Test_init(loop);
		if (testResult == SDL_PASS)
		{
			/* Run the test for diagnostics first */
			testResult = CCM_runTest(loop);
		}
	}

    return (testResult);
}

int32_t cpu_example_app(uint32_t ccmcore)
{
    int32_t       testResult = 0;
    SDL_ErrType_t sdlResult;

    sdlResult = SDL_TEST_CCMSelfTest(ccmcore);
	if (sdlResult != SDL_PASS)
	{
		/* CCM Self test is not passed*/
		testResult = SDL_EFAIL;
	}
	if(testResult == SDL_PASS)
	{
        sdlResult = SDL_TEST_CCMSelfTestErrorForce(ccmcore);
    	if (sdlResult != SDL_PASS)
    	{
    	 	/* CCM seft test with error force is not passed */
    		testResult = SDL_EFAIL;
    	}
	}
	if(testResult == SDL_PASS)
	{
        sdlResult = SDL_TEST_CCMInjectError(ccmcore);
        if (sdlResult != SDL_PASS)
    	{
            /* CCM Error inject test is not passed */
            testResult = SDL_EFAIL;
        }
    }
	return (testResult);

}

void ccm_clearESM(uint32_t intSrc)
{
    SDL_CCM_MonitorType monitorType;
    /* get monitor type from error, Here for both insatnce, baseaddress is same */
    SDL_CCM_getErrorType(SDL_R5SS0_CCM, intSrc, &monitorType);
    /* clear esm error, Here for both insatnce, baseaddress is same */
    SDL_CCM_clearError(SDL_R5SS0_CCM, monitorType);

}

/* ========================================================================== */
/*                            Internal Function Definition                    */
/* ========================================================================== */


/* Nothing past this point */
