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
 *  \file     parity_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the TCM Parity application.
 *
 *  \details  TCM Parity Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include "parity_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This macro shows how many ESM events are configured*/
#define SDL_ESM_MAX_EXAMPLE                (6u)
#define SDL_INTR_GROUP_NUM_2               (2U)
#define SDL_INTR_PRIORITY_LVL_LOW          (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH         (1U)
#define SDL_ENABLE_ERR_PIN                 (1U)
/**
*	\brief  Developer can enable or disable below macros for the testing off
*           TCM different banks
**/
#define SDL_ATCM0_MASK							(0x7U)
#define SDL_ATCM1_MASK							(0x70U)
#define SDL_B0TCM0_MASK							(0x700U)
#define	SDL_B0TCM1_MASK							(0x7000U)
#define SDL_B1TCM0_MASK							(0x70000U)
#define	SDL_B1TCM1_MASK							(0x700000U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

/* Event BitMap for ESM callback  */
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_EXAMPLE] =
{
	/* ATCM */
    {
		/* Event BitMap for ESM callback for ATCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_ATCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_ATCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* B0TCM */
	{
		/* Event BitMap for ESM callback for B0TCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B0TCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B0TCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* B1TCM */
	{
		/* Event BitMap for ESM callback for B0TCM0 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B1TCM0_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	{
		/* Event BitMap for ESM callback for ATCM1 Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_2,
		.errorNumber = SDL_ESMG2_B1TCM1_PARITY_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },

};
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
 * @fn      Parity_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t Parity_sdlFuncTest(void)
{
    int32_t result=SDL_EFAIL;
    int32_t retVal = 0u;

	DebugP_log("\r\nTCM PARITY Functional Test : Started\r\n");
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[0],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
    if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: ATCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_TCM_PARITY_ATCM0,\
								 SDL_ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS ATCM0 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS ATCM0 Parity : Completed\r\n");
		}
	}
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[1],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
	if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: ATCM1 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_TCM_PARITY_ATCM1,\
								 SDL_ATCM1_MASK);
		/* Wait until ESM interrupt happens */						 
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS ATCM1 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS ATCM1 Parity : Completed \r\n");
		}
	}
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[2],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
	if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: B0TCM0 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_TCM_PARITY_B0TCM0,\
								 SDL_B0TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS B0TCM0 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS B0TCM0 Parity : Completed \r\n");
		}
	}
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[3],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
	if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: B0TCM1 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_TCM_PARITY_B0TCM1,\
								 SDL_B0TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS B0TCM1 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS B0TCM1 Parity : Completed \r\n");
		}
	}
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[4],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
	if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: B1TCM0 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_TCM_PARITY_B1TCM0,\
								 SDL_B1TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS B1TCM0 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS B1TCM0 Parity : Completed \r\n");
		}
	}
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[5],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
	
	if (retVal == 0)
    {		
		DebugP_log("\r\nMSS TCM PARITY: B1TCM1 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_TCM_PARITY_B1TCM1,\
								 SDL_B1TCM1_MASK);							 
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;		
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nMSS B1TCM1 Parity : Failed\r\n");
		}
		else{
			retVal = 0;
			DebugP_log("\r\nMSS B1TCM1 Parity : Completed \r\n");
		}
	}

    return retVal;
}

/*********************************************************************
 * @fn      sdlApp_dplInit
 *
 * @brief   Initialization of DPL
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }

    return ret;
}

/* ECC Function module test */
int32_t Parity_funcTest(void)
{
    int32_t testResult = 0;

    /*Initializing the DPL*/
    sdlApp_dplInit();

    /*Execute ECC sdl function test*/
    testResult = Parity_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
