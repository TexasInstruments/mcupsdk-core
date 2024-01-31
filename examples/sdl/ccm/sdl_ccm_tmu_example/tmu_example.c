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
 *  \file     tmu_example.c
 *
 *  \brief    This file contains R5F-CPU Safety Example showing usage of R5F
 *  TMU Lockstep as a safety mechanism to detect errors in R5F execution.
 *  Example shows how to configure and use all 4 modes. *
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
#include "tmu_main.h"
#include <sdl/sdl_ecc.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* ========================================================================== */
/*                            Internal Function Declaration                    */
/* ========================================================================== */
/*********************************************************************
 * @fn      SDL_TEST_TMUSelfTest
 *
 * @brief   Execute CCM TMU Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t tmu_example_app(uint32_t instance);

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                                   SDL_ESM_IntType esmIntrType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint32_t ESM_arg;
volatile bool ESMError = false;

SDL_ESM_config CCM_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */

    .enableBitmap = {0x00000000u, 0x00000000u, 0xFFFFFFFFu, /*0x00800000u,*/ 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x00000000u, 0x00000000u, 0xFFFFFFFFu, 0x00000000u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0xFFFFFFFFu, 0x00000000u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t SDL_TEST_TMUSelfTest(uint32_t ccmInstance)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("TMU self test: starting\r\n");

    result = SDL_CCM_selfTest(ccmInstance,
	                          SDL_CCM_MONITOR_TYPE_TMU,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS )
    {
        DebugP_log("TMU self test failed\r\n");
        retVal = -1;
    }
    else
    {
        DebugP_log("TMU Self Test completed\r\n");
    }

    return retVal;
}


/*********************************************************************
 * @fn      SDL_TEST_TMUSelfTestErrorForce
 *
 * @brief   Execute CCM Self test with error force
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_TMUSelfTestErrorForce(uint32_t InstanceCCM)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("TMU self test with error forcing: starting\r\n");

    result = SDL_CCM_selfTest(InstanceCCM,
	                          SDL_CCM_MONITOR_TYPE_TMU,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS )
    {
        DebugP_log("TMU self test with error forcing failed\r\n");
        retVal = -1;
    }
    else
    {
        DebugP_log("TMU Self Test with error forcing completed\r\n");
    }

    return retVal;
}

/*********************************************************************
 * @fn      SDL_TEST_TMUInjectError
 *
 * @brief   Execute CCM Inject Error test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_TMUInjectError(uint32_t ccmInstanceID)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("TMU inject  error: test starting\r\n");

    result = SDL_CCM_injectError(ccmInstanceID, SDL_CCM_MONITOR_TYPE_TMU);

    if (result != SDL_PASS )
    {
        DebugP_log("TMU inject error failed\r\n");
       retVal = -1;
    }
    else
    {
        DebugP_log("TMU inject Test completed\r\n");
    }

    return retVal;
}

static int32_t CCM_runTest(uint32_t instanceId)
{
    int32_t       testResult = 0;

	testResult = tmu_example_app(instanceId);
    return (testResult);
}


int32_t CCM_Test_init (int32_t instNum)
{
    int32_t result=0, retValue=0;
    void *ptr = (void *)&ESM_arg;

    /* Initialize MAIN ESM module */
    result = SDL_ESM_init(ESM_INSTANCE, &CCM_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);

    if (result != SDL_PASS)
    {
        /* print error and quit */
         DebugP_log("TMU_Test_init: Error initializing MCU ESM: result = %d\r\n", result);
        retValue = -1;
    }
    else
    {
        DebugP_log("MCU ESM Initialization completed\r\n");
    }

    if (retValue == 0)
    {

        /* Initialize CCM */
        result = SDL_CCM_init(instNum,0);
        if (result != SDL_PASS)
        {
            /* print error and quit */
            DebugP_log("TMU_Test_init: Error result = %d\n", result);
            retValue = -1;
        }
        else
        {
            DebugP_log("TMU Test Init completed\r\n");
        }
        /* Initialize CCM */
        result = SDL_CCM_verifyConfig(instNum);
        if (result != SDL_PASS)
        {
            /* print error and quit */
            DebugP_log("TMU_Verify_Test_init: Error result = %d\r\n", result);
            retValue = -1;
        }
        else
        {
            DebugP_log("TMU Verify Init completed\r\n");
        }
    }

    return retValue;
}

/* CCM Functional test */
int32_t CCM_funcTest(void)
{
    int32_t    testResult = 0;
	int32_t    loop=2;
    int32_t    loopCnt=2;

	for(loop=0; loop < loopCnt; loop++)
	{
		DebugP_log("TMU Example Test Started: R5F%d\r\n",loop);
		testResult = CCM_Test_init(loop);
		if (testResult != 0)
		{
			DebugP_log("TMU SDL API tests: unsuccessful\r\n");
			return SDL_EFAIL;
		}
		if (testResult == SDL_PASS)
		{
			DebugP_log("TMU Functional Test \r\n");
			/* Run the test for diagnostics first */
			testResult = CCM_runTest(loop);
		}
		else
		{
			DebugP_log("TMU Init failed. Exiting the app\r\n");
		}
	}

    return (testResult);
}

int32_t tmu_example_app(uint32_t ccmcore)
{
    int32_t       testResult = 0;
    SDL_ErrType_t sdlResult;

    sdlResult = SDL_TEST_TMUSelfTest(ccmcore);
	if (sdlResult != SDL_PASS)
	{
		DebugP_log("TMU SelfTest: failure on line no. %d \n", __LINE__);
		testResult = -1;
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_TMUSelfTestErrorForce(ccmcore);
    	if (sdlResult != SDL_PASS)
    	{
    	 	DebugP_log("TMU SelfTest with error force: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_TMUInjectError(ccmcore);
        if (sdlResult != SDL_PASS)
    	{
            DebugP_log("SDL_TEST_TMUInjectError: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
	return (testResult);

}
/* ========================================================================== */
/*                            Internal Function Definition                    */
/* ========================================================================== */
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
											void *ESM_arg)
{
    int32_t retVal = 0;
    SDL_CCM_MonitorType monitorType;
    printf("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("  Take action \n");

    SDL_CCM_getErrorType(SDL_R5SS0_CCM, intSrc, &monitorType);
    SDL_CCM_clearError(SDL_R5SS0_CCM, monitorType);

   return retVal;
}
/* Nothing past this point */
