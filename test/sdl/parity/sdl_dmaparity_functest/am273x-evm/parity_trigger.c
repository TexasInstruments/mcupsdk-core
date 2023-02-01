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
 *  \file     parity_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the DMA Parity application.
 *
 *  \details  DMA Parity Safety Example module tests
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
#include <sdl/include/am273x/sdlr_mss_param_regs.h>
#include <sdl/include/am273x/sdlr_dss_param_regs.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This macro shows how many ESM events are configured*/
#define SDL_ESM_MSS_MAX_EXAMPLE                (2u)
#define SDL_ESM_DSS_MAX_EXAMPLE                (3u)

#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)


/* PARAM Registers */
/* MSS */
#define SDL_MSS_TPCCA_PARAM_REG1					(SDL_MSS_PARAM_REG_A_SET0 + 0x20U)
#define SDL_MSS_TPCCB_PARAM_REG1					(SDL_MSS_PARAM_REG_B_SET0 + 0x20U)
/*DSS */
#define SDL_DSS_TPCCA_PARAM_REG1					(SDL_DSS_PARAM_REG_A_SET0 + 0x20U)
#define SDL_DSS_TPCCB_PARAM_REG1					(SDL_DSS_PARAM_REG_B_SET0 + 0x20U)
#define SDL_DSS_TPCCC_PARAM_REG1					(SDL_DSS_PARAM_REG_C_SET0 + 0x20U)
/* value to write into param register */
#define PARAMSET_VAL_1					(0x7U)
/* Enable testmode and parity in TPCC_PARITY_CTRL register */ 
#define PARITY_MSS_A_ENABLE    0x11u
#define PARITY_MSS_B_ENABLE    0x1100u
#define PARITY_DSS_ENABLE      0x3u
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

/* Event BitMap for ESM callback for MSS */
SDL_ESM_NotifyParams ECC_Testparams[SDL_ESM_MSS_MAX_EXAMPLE] =
{
	/* MSS TPTC */
    {
		/* Event BitMap for ESM callback for TPCC_A Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_MSS_TPCC_A_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* MSS TPTC */
    {
		/* Event BitMap for ESM callback for TPCC_B Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_ESMG1_MSS_TPCC_B_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	

};
SDL_ESM_NotifyParams ECC_TestparamsDSS[SDL_ESM_DSS_MAX_EXAMPLE] =
{
	/* DSS TPTCA */
    {
		/* Event BitMap for ESM callback for TPCC_A Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_A_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* DSS TPTCB */
    {
		/* Event BitMap for ESM callback for TPCC_B Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_B_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
		.enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
		.callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	/* DSS TPTCC */
    {
		/* Event BitMap for ESM callback for TPCC_C Single bit*/
		.groupNumber = SDL_INTR_GROUP_NUM_1,
		.errorNumber = SDL_DSS_ESMG1_DSS_TPCC_C_INTAGG_ERR,
		.setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
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
    int32_t result = SDL_EFAIL;
    int32_t retVal = 0u;
	int32_t paramstatus=0u;
	int32_t i=0;
	
#if defined(R5F_INPUTS)
	DebugP_log("\r\nMSS TPCC PARITY Functional Test : Started\r\n");
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_Testparams[0],NULL,NULL);
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
		DebugP_log("\r\nMSS TPCCA Parity \r\n");
        paramstatus = SDL_ECC_tpccParity(SDL_TPCC0A, \
								 PARITY_MSS_A_ENABLE, SDL_MSS_TPCCA_PARAM_REG1, PARAMSET_VAL_1);	
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);
		
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		/*wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );
		
		if(esmError == true)
		{
			DebugP_log("\r\nMSS TPCCA Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nMSS TPCCA Parity : Failed\r\n");
		}
	}
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_Testparams[1],NULL,NULL);
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
		DebugP_log("\r\nMSS TPCCB Parity \r\n");
        paramstatus = SDL_ECC_tpccParity(SDL_TPCC0B, \
								 PARITY_MSS_B_ENABLE, SDL_MSS_TPCCB_PARAM_REG1, PARAMSET_VAL_1);	
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);
		
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		/*wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );
		
		if(esmError == true)
		{
			DebugP_log("\r\nMSS TPCCB Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nMSS TPCCB Parity : Failed\r\n");
		}
	}
#endif
#if defined(C66_INPUTS)
	DebugP_log("\r\nDSS TPCC PARITY Example : Started\r\n");
	
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init DSS ESM complete \r\n");
        }
    }
	
    if (retVal == 0)
    {		
		DebugP_log("\r\nDSS TPCCA Parity \r\n");
        paramstatus = SDL_ECC_tpccParity(SDL_DSS_TPCCA, \
								 PARITY_DSS_ENABLE, SDL_DSS_TPCCA_PARAM_REG1, PARAMSET_VAL_1);	
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);
		
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		/*wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );
		
		if(esmError == true)
		{
			DebugP_log("\r\nDSS TPCCA Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nDSS TPCCA Parity : Failed\r\n");
		}
	}
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init DSS ESM complete \r\n");
        }
    }
	
    if (retVal == 0)
    {		
		DebugP_log("\r\nDSS TPCCB Parity \r\n");
        paramstatus = SDL_ECC_tpccParity(SDL_DSS_TPCCB, \
								 PARITY_DSS_ENABLE, SDL_DSS_TPCCB_PARAM_REG1, PARAMSET_VAL_1);
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);
		
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		
		for (i =0 ; i < 200 ; i = i + 1 );
		
		if(esmError == true)
		{
			DebugP_log("\r\nDSS TPCCB Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nDSS TPCCB Parity : Failed\r\n");
		}
	}
	if (retVal == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[2],NULL,NULL);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init DSS ESM complete \r\n");
        }
    }
	
    if (retVal == 0)
    {		
		DebugP_log("\r\nDSS TPCCC Parity \r\n");
        paramstatus = SDL_ECC_tpccParity(SDL_DSS_TPCCC, \
								 PARITY_DSS_ENABLE, SDL_DSS_TPCCC_PARAM_REG1, PARAMSET_VAL_1);
		DebugP_log("\r\nParam Register = %x\r\n", paramstatus);
		
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		
		for (i =0 ; i < 200 ; i = i + 1 );
		
		if(esmError == true)
		{
			DebugP_log("\r\nDSS TPCCC Parity : Completed\r\n");
			esmError = false;
		}
		else{
			retVal = -1;
			DebugP_log("\r\nDSS TPCCC Parity : Failed\r\n");
		}
	}
#endif		

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
