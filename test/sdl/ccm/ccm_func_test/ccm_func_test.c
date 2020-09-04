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
 *  \file     ccm_func_test.c
 *
 *  \brief    This file contains CCM Functional test code.
 *
 *  \details  CCM Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include "ccm_func_main.h"
#include "sdl/r5/v0/sdl_mcu_armss_ccmr5.h"
#include <dpl_interface.h>
#if defined(SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#endif
#if defined(SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/esm/soc/am273x/sdl_esm_core.h>
#include <sdl/include/am273x/sdlr_intr_esm_mss.h>
#endif
#if defined(SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/esm/soc/awr294x/sdl_esm_core.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#endif
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
#if defined (SOC_AM263X)
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
int32_t SDL_ESM_applicationCallback(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg);
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (SOC_AM263X)
SDL_ESM_config CCM_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */

    .enableBitmap = {0x00000000u, 0x00000000u, 0x00004000u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00004000u, 0x00000000u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00004000u, 0x00000000u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};
#endif
#if defined (SOC_AM273X) || defined(SOC_AWR294X)
SDL_ESM_NotifyParams SDL_CCM_eventBitMap[SDL_ESM_MAX_EVENT_MAP_NUM_WORDS] =
{
    {
          /* Event BitMap for CCM ESM callback */
          .groupNumber = SDL_INTR_GROUP_NUM,
          .errorNumber = SDL_ESMG2_CCMR5_COMPARE,//25
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL,//1
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallback,
     }
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t arg;
volatile bool ESMError = false;

/*********************************************************************
 * @fn      SDL_TEST_CCMSelfTest
 *
 * @brief   Execute CCM Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMSelfTest(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM self test: starting");

    result = SDL_CCM_selfTest(INSTANCE,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM self test failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Self Test complete");
    }

    return retVal;
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
int32_t SDL_TEST_CCMSelfTestErrorForce(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM self test with error forcing: starting");

    result = SDL_CCM_selfTest(INSTANCE,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM self test with error forcing failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Self Test with error forcing complete");
    }

    return retVal;
}

int32_t SDL_TEST_CCMSelfTest_Inactivity_ErrorForce(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM Inactivity self test with error forcing: starting");

    result = SDL_CCM_selfTest(INSTANCE,
                              SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM Inactivity self test with error forcing failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Inactivity Self Test with error forcing complete");
    }

    return retVal;
}

int32_t SDL_TEST_CCMSelfTest_VIM_ErrorForce(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM VIM self test with error forcing: starting");

    result = SDL_CCM_selfTest(INSTANCE,
                              SDL_CCM_MONITOR_TYPE_VIM,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM VIM self test with error forcing failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM VIM Self Test with error forcing complete");
    }

    return retVal;
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
int32_t SDL_TEST_CCMInjectError(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject  error: test starting");

    result = SDL_CCM_injectError(INSTANCE, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject failed");
       retVal = -1;
    } else {
        DebugP_log("\n CCM inject Test complete");
    }

    return retVal;
}

int32_t SDL_TEST_CCMInjectVIMError(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject VIM error: test starting");

    result = SDL_CCM_injectError(INSTANCE, SDL_CCM_MONITOR_TYPE_VIM);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject VIM failed");
       retVal = -1;
    } else {
        DebugP_log("\n CCM inject VIM Test complete");
    }

    return retVal;
}

int32_t SDL_TEST_CCMInjectInactivityError(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject inactivity monitor error: test starting");

    result = SDL_CCM_injectError(INSTANCE, SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject failed");
       retVal = -1;
    } else {
        DebugP_log("\n CCM inject inactivity monitor Test complete");
    }

    return retVal;
}

/*********************************************************************
 * @fn      SDL_TEST_CCMSelftestPolarityInvert
 *
 * @brief   Execute CCM with polarity inversion
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMSelftestPolarityInvert(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM polarity invert self test: starting");

    result = SDL_CCM_selfTest(INSTANCE,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_POLARITY_INVERSION, 0xFFU,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM polarity invert self test failed");
       retVal = -1;
    } else {
        DebugP_log("\n CCM polarity invert self test complete");
    }

    return retVal;
}

/*********************************************************************
 * @fn      SDL_TEST_CCMVIMSelfTest
 *
 * @brief   Execute CCM VIM Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMVIMSelfTest(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM VIM self test: starting");

    result = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_VIM,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM VIM self test failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM VIM Self Test complete");
    }

    return retVal;
}

/*********************************************************************
 * @fn      SDL_TEST_CCMInactivitySelfTest
 *
 * @brief   Execute CCM inactivity monitor Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDL_TEST_CCMInactivitySelfTest(void)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inactivity monitor self test: starting");

    result = SDL_CCM_selfTest(INSTANCE, SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inactivity monitor self test failed");
        retVal = -1;
    } else {
        DebugP_log("\n CCM inactivity monitor Self Test complete");
    }

    return retVal;
}

int32_t CCM_runTest(uint32_t instanceId)
{
    int32_t       testResult = 0;
    SDL_ErrType_t sdlResult;

    if(testResult == 0)
    {
        sdlResult = SDL_TEST_CCMSelfTest();
	if (sdlResult != SDL_PASS)
	{
		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
		testResult = -1;
	}
    }
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTestErrorForce();
    	if (sdlResult != SDL_PASS)
    	{
    	 	DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectError();
        if (sdlResult != SDL_PASS)
    	{
            DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_TEST_CCMInactivitySelfTest();
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
				testResult = -1;
			}
    	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTest_Inactivity_ErrorForce();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectInactivityError();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelftestPolarityInvert();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMVIMSelfTest();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTest_VIM_ErrorForce();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectVIMError();
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_CCM_injectError(instanceId, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \n", __LINE__);
    		testResult = -1;
    	}
	}

    return (testResult);
}


int32_t CCM_Test_init (void)
{
    int32_t result, retValue=0;
    void *ptr = (void *)&arg;

    if (retValue == 0) {
#if defined(SOC_AM263X)
        result = SDL_ESM_init(ESM_INSTANCE, &CCM_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
#endif
#if defined(SOC_AM273X)||defined(SOC_AWR294X)
        result = SDL_ESM_init(ESM_INSTANCE, &SDL_CCM_eventBitMap[0], NULL, ptr);
#endif
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error initializing MAIN ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: Init MAIN ESM complete \n");
        }
    }

    if (retValue == 0) {
        /* Initialize CCM */
        result = SDL_CCM_init(0);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: CCM Init complete \n");
        }
		        /* Initialize CCM */
        result = SDL_CCM_verifyConfig(0);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: CCM Init complete \n");
        }
    }

    return retValue;
}

/* CCM Functional test */
int32_t CCM_funcTest(void)
{
    int32_t    testResult = 0;

    testResult = CCM_Test_init();

    if (testResult != 0)
    {
        DebugP_log("\n CCM SDL API tests: unsuccessful");
        return SDL_EFAIL;
    }

	if (testResult == SDL_PASS)
	{
		DebugP_log("\nCCM Functional Test \r\n");
	    /* Run the test for diagnostics first */
		testResult = CCM_runTest(INSTANCE);
	}
	else
	{
		DebugP_log("\r\nCCM Init failed. Exiting the app.\r\n");
	}


    return (testResult);
}
/* ========================================================================== */
/*                            Internal Function Definition                    */
/* ========================================================================== */
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
int32_t SDL_ESM_applicationCallback(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    
    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    
    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    ESMError = true;

    return retVal;
}
#endif
/* Nothing past this point */
