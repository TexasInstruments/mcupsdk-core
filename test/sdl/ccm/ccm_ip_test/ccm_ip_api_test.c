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
 *  \file     ccm_ip_api_test.c
 *
 *  \brief    This file contains CCM IP API test code.
 *
 *  \details  CCM IP tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/sdl_types.h>
#include "ccm_test_main.h"
#include <sdl/r5/v0/sdl_mcu_armss_ccmr5.h>

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


static int32_t CCM_IP_test(uint32_t instanceId)
{
    int32_t       testResult = 0;
    SDL_ErrType_t sdlResult;
    int i;
	uint32_t    valToBeRead;
	int32_t     metaInfo;
    SDL_McuArmssCcmR5OpModeKey    opModeKey;
    uint32_t      cmpError;

    for(i = SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID; i <= 	SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID; i++)
	{
		if(testResult == 0)
		{
			sdlResult = SDL_armR5ReadCCMRegister(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5RegId)i, &valToBeRead, &metaInfo);
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
				testResult = -1;
				break;
			}
		}

		if(testResult == 0)
		{
			sdlResult = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5RegId)i, valToBeRead, &metaInfo);
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
				testResult = -1;
				break;
			}	
		}
	}
	
	if(testResult == 0)
	{
		sdlResult = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instanceId], SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID, valToBeRead, NULL);
		if (sdlResult != SDL_PASS)
		{
			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}	
	}
	
    for(i = SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID; i <= SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID; i++)
	{
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}	
    	}
    	
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            &opModeKey, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}
    	}
    
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}	
    	}
    	
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            &opModeKey, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}
    	}
    	
    	if(testResult == 0)
    	{
    		(void)SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE, &metaInfo);
    		sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_ERR_FORCE_MODE, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}	
    	}
    	
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            &opModeKey, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}
    	}
    
    	if(testResult == 0)
    	{
    		(void)SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE, &metaInfo);
    		sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_ERR_FORCE_MODE, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}	
    	}
    	
    	if(testResult == 0)
    	{
    		sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, 
			                                            &opModeKey, &metaInfo);
    		if (sdlResult != SDL_PASS)
    		{
    			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
    			testResult = -1;
    		}
    	}
    }
	
	if(testResult == 0)
	{
		sdlResult = SDL_armR5CCMGetOperationModeKey(SDL_CCM_baseAddress[instanceId], SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID, &opModeKey, NULL);
		if (sdlResult != SDL_PASS)
		{
			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
	{
		sdlResult = SDL_armR5CCMSetOperationModeKey(SDL_CCM_baseAddress[instanceId], SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID, opModeKey, NULL);
		if (sdlResult != SDL_PASS)
		{
			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}	
	}
	
	if(testResult == 0)
	{
		for(i = SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID; i <= 	SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID; i++)
		{
			sdlResult = SDL_armR5CCMGetCompareError(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, &cmpError, &metaInfo);
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
				testResult = -1;
				break;
			}
		}
	}
	
	if(testResult == 0)
	{
		sdlResult = SDL_armR5CCMGetCompareError(SDL_CCM_baseAddress[instanceId], SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID, \
                                          		&cmpError, NULL);
		if (sdlResult != SDL_PASS)
		{
			DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
			testResult = -1;
		}
	}
	
	if(testResult == 0)
	{
		for(i = SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID; i <= 	SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID; i++)
		{
			sdlResult = SDL_armR5CCMClearCompareError(SDL_CCM_baseAddress[instanceId], (SDL_McuArmssCcmR5ModuleId)i, &metaInfo);
			if (sdlResult != SDL_PASS)
			{
				DebugP_log("sdlCcm_apiTest: failure on line no. %d \n", __LINE__);
				testResult = -1;
				break;
			}
		}
	}

    return (testResult);
}

/* CCM Functional test */
int32_t CCM_ipApiTest(void)
{
    int32_t    testResult = 0;
    int i = 0;

	/* Run the test for diagnostics first */
	/* Run test on selected instance */
	testResult = CCM_IP_test(i);

    return (testResult);
}
/* Nothing past this point */
