/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <sdl/r5/v0/sdl_ip_ccm.h>
#include <sdl/include/sdl_types.h>
#include <sdl/r5/v0/sdl_mcu_armss_ccmr5.h>
#include <dpl_interface.h>
#include <sdl/sdl_ecc.h>
#if defined(SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#endif
#if defined (SOC_AM263PX) || defined (SOC_AM261X)
#include <sdl/esm/v2/sdl_esm.h>
#endif
#if defined(SOC_AM273X)
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
#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define INSTANCE 		SDL_R5SS0_CCM
#elif defined(SOC_AM273X) || defined(AWR294X)
#define INSTANCE 		SDL_MSS_CCMR
#endif
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config CCM_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */

    .enableBitmap = {0x00000000u, 0x00000000u, 0x00780880u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00780880u, 0x00000000u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00780880u, 0x00000000u,
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
        .errorNumber = SDL_ESMG1_CCMR5_ST_ERR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ESM_applicationCallback,
    },
	{
        /* Event BitMap for CCM ESM callback */
        .groupNumber = SDL_INTR_GROUP_NUMBER,
        .errorNumber = SDL_ESMG2_VIM_LOCK_ERR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ESM_applicationCallback,
    },
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
int32_t SDL_TEST_CCMSelfTest(uint32_t ccmcore)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM self test: starting\r\n");

    result = SDL_CCM_selfTest(ccmcore,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM self test failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Self Test complete\r\n");
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
int32_t SDL_TEST_CCMSelfTestErrorForce(uint32_t INSTID)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM self test with error forcing: starting\r\n");

    result = SDL_CCM_selfTest(INSTID,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM self test with error forcing failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Self Test with error forcing complete\r\n");
    }

    return retVal;
}

int32_t SDL_TEST_CCMSelfTest_Inactivity_ErrorForce(uint32_t CORE_INST)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM Inactivity self test with error forcing: starting\r\n");

    result = SDL_CCM_selfTest(CORE_INST,
                              SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM Inactivity self test with error forcing failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM Inactivity Self Test with error forcing complete\r\n");
    }

    return retVal;
}

int32_t SDL_TEST_CCMSelfTest_VIM_ErrorForce(uint32_t Instance_CORE)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM VIM self test with error forcing: starting\r\n");

    result = SDL_CCM_selfTest(Instance_CORE,
                              SDL_CCM_MONITOR_TYPE_VIM,
                              SDL_CCM_SELFTEST_TYPE_ERROR_FORCING, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM VIM self test with error forcing failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM VIM Self Test with error forcing complete\r\n");
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
int32_t SDL_TEST_CCMInjectError(uint32_t InstanceCCM)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject  error: test starting\r\n");

    result = SDL_CCM_injectError(InstanceCCM, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM inject Test complete\r\n");
    }

    return retVal;
}

int32_t SDL_TEST_CCMInjectVIMError(uint32_t InstanceIDccm)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject VIM error: test starting\r\n");

    result = SDL_CCM_injectError(InstanceIDccm, SDL_CCM_MONITOR_TYPE_VIM);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject VIM failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM inject VIM Test complete\r\n");
    }

    return retVal;
}

int32_t SDL_TEST_CCMInjectInactivityError(uint32_t Instccm)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inject inactivity monitor error: test starting\r\n");

    result = SDL_CCM_injectError(Instccm, SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inject failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM inject inactivity monitor Test complete\r\n");
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
int32_t SDL_TEST_CCMSelftestPolarityInvert(uint32_t ccmInst)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM polarity invert self test: starting\r\n");

    result = SDL_CCM_selfTest(ccmInst,
	                          SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK,
                              SDL_CCM_SELFTEST_POLARITY_INVERSION, 0xFFU,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM polarity invert self test failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM polarity invert self test complete\r\n");
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
int32_t SDL_TEST_CCMVIMSelfTest(uint32_t ccmInstance)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM VIM self test: starting\r\n");

    result = SDL_CCM_selfTest(ccmInstance, SDL_CCM_MONITOR_TYPE_VIM,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM VIM self test failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM VIM Self Test complete\r\n");
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
int32_t SDL_TEST_CCMInactivitySelfTest(uint32_t ccmInstanceID)
{
    int32_t result;
    int32_t retVal=0;

    DebugP_log("\n CCM inactivity monitor self test: starting\r\n");

    result = SDL_CCM_selfTest(ccmInstanceID, SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR,
                              SDL_CCM_SELFTEST_TYPE_NORMAL, 0U,
                              10000000);

    if (result != SDL_PASS ) {
        DebugP_log("\n CCM inactivity monitor self test failed\r\n");
        retVal = -1;
    } else {
        DebugP_log("\n CCM inactivity monitor Self Test complete\r\n");
    }

    return retVal;
}

static int32_t CCM_runTest(uint32_t instanceId)
{
    int32_t       testResult = 0;
    SDL_ErrType_t sdlResult;

    if(testResult == 0)
    {
        sdlResult = SDL_TEST_CCMSelfTest(instanceId);
	if (sdlResult != SDL_PASS)
	{
		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
		testResult = -1;
	}
    }
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTestErrorForce(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    	 	DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectError(instanceId);
        if (sdlResult != SDL_PASS)
    	{
            DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
            testResult = -1;
        }
    }
    if(testResult == 0)
    {
        sdlResult = SDL_TEST_CCMInactivitySelfTest(instanceId);
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
				testResult = -1;
			}
    	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTest_Inactivity_ErrorForce(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectInactivityError(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelftestPolarityInvert(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMVIMSelfTest(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMSelfTest_VIM_ErrorForce(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_TEST_CCMInjectVIMError(instanceId);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}
	if(testResult == 0)
	{
        sdlResult = SDL_CCM_injectError(instanceId, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);
    	if (sdlResult != SDL_PASS)
    	{
    		DebugP_log("sdlCcm_funcTest: failure on line no. %d \r\n", __LINE__);
    		testResult = -1;
    	}
	}

    return (testResult);
}


int32_t CCM_Test_init (int32_t instNum, uint32_t indexNum)
{
    int32_t result, retValue=0;
    void *ptr = (void *)&arg;

    if (retValue == 0) {
#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
        result = SDL_ESM_init(ESM_INSTANCE, &CCM_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
#endif
#if defined(SOC_AM273X)||defined(SOC_AWR294X)
        result = SDL_ESM_init(ESM_INSTANCE, &SDL_CCM_eventBitMap[indexNum], NULL, ptr);
#endif
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error initializing MAIN ESM: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: Init MAIN ESM complete \r\n");
        }
    }

    if (retValue == 0) {
        /* Initialize CCM */
        result = SDL_CCM_init(instNum, indexNum);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: CCM Init complete\r\n");
        }
		        /* Initialize CCM */
        result = SDL_CCM_verifyConfig(instNum);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("CCM_Test_init: Error result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nCCM_Test_init: CCM Init complete\r\n");
        }
    }

    return retValue;
}

/* CCM Functional test */
int32_t CCM_funcTest(void)
{
    int32_t    testResult = 0;
	int32_t    loop= 0;
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
	int32_t loopCnt=2;
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
	int32_t loopCnt=1;
#endif
	for(loop=0; loop < loopCnt; loop++)
	{
		testResult = CCM_Test_init(loop, loop);
		DebugP_log("CCM Example Test Started: R5F%d\r\n",loop);

		if (testResult != 0)
		{
			DebugP_log("\n CCM SDL API tests: unsuccessful\r\n");
			return SDL_EFAIL;
		}

		if (testResult == SDL_PASS)
		{
			DebugP_log("\nCCM Functional Test \r\n");
			/* Run the test for diagnostics first */
			testResult = CCM_runTest(loop);
		}
		else
		{
			DebugP_log("\r\nCCM Init failed. Exiting the app.\r\n");
		}
	}
	Board_driversClose();
    Drivers_close();

	return (testResult);
}

void func_test_main(void *args)
{
	SDL_ErrType_t ret = SDL_PASS;

	Drivers_open();
    Board_driversOpen();

    ret = SDL_TEST_dplInit();

    if (ret != SDL_PASS)
    {
        DebugP_log("Error: DPL Init Failed\r\n");
    }
	ret = CCM_funcTest();
	if (ret != SDL_PASS)
    {
        DebugP_log("Error: Function Test Failed\r\n");
    }

}
/* ========================================================================== */
/*                            Internal Function Definition                    */
/* ========================================================================== */
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg)
{

    int32_t retVal = 0;
    SDL_CCM_MonitorType monitorType;
    DebugP_log("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInstType, esmIntType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");

    SDL_CCM_getErrorType(INSTANCE, intSrc, &monitorType);
    SDL_CCM_clearError(INSTANCE, monitorType);

   return retVal;
}
#endif
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
