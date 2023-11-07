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
 *  \details  TCM Parity Safety Example module tests
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
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/**
*	\brief  Developer can enable or disable below macros for the testing off
*           TCM different banks
**/
#define SDL_R5F0ATCM0_MASK							(0x7U)
#define SDL_R5F0B0TCM0_MASK							(0x700U)
#define	SDL_R5F0B1TCM0_MASK							(0x70000U)

#define SDL_R5F0ATCM1_MASK							(0x70U)
#define SDL_R5F0B0TCM1_MASK							(0x7000U)
#define	SDL_R5F0B1TCM1_MASK							(0x700000U)

#define SDL_R5F1ATCM0_MASK							(0x7U)
#define SDL_R5F1B0TCM0_MASK							(0x700U)
#define	SDL_R5F1B1TCM0_MASK							(0x70000U)

#define SDL_R5F1ATCM1_MASK							(0x70U)
#define SDL_R5F1B0TCM1_MASK							(0x7000U)
#define	SDL_R5F1B1TCM1_MASK							(0x700000U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint32_t arg;
SDL_ESM_config Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x0003C000u, 0x00000000u, 0x00000010u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events enable: except clkstop events for unused clocks
    *   and PCIE events */
    .priorityBitmap = {0x0000C000u, 0x000000000u, 0x00000010u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
    /**< All events high priority: except clkstop events for unused clocks
    *   and PCIE events */
    .errorpinBitmap = {0x0003C000u, 0x00000000u, 0x00000010u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events high priority: except clkstop for unused clocks
    *   and PCIE events */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Parity_Example_init function */
int32_t Parity_Example_init (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      Parity_Example_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t Parity_Example_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
    void *ptr = (void *)&arg;

	if (retValue == 0)
    {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS)
        {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retValue = -1;
        }
        else
        {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
    return retValue;
}

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
    int32_t retVal = 0;
	int32_t result= SDL_PASS;

	DebugP_log("\r\nTCM PARITY Example : Started\r\n");
	DebugP_log("\r\nTCM PARITY : R5FSS0_0\r\n");

	/* R5FSS0_0 */
	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : ATCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_ATCM0,\
								 SDL_R5F0ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for ATCM0 = 0x%x\r\n", statusRegValue0);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_0 ATCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_0 ATCM0 Parity : Completed\r\n");
		}
	}
	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B0TCM0 Started\r\n");
		result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_B0TCM0,\
								 SDL_R5F0B0TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B0TCM0 = 0x%x\r\n", statusRegValue0);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_0 B0TCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_0 B0TCM0 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B1TCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_B1TCM0,\
								 SDL_R5F0B1TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B1TCM0 = 0x%x\r\n", statusRegValue0);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_0 B1TCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_0 B1TCM0 Parity : Completed\r\n");
		}
	}

/* R5FSS0_1 */

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY:R5FSS0_1\r\n");
		DebugP_log("\r\nMSS TCM PARITY:ATCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_ATCM1,\
								 SDL_R5F0ATCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for ATCM1 = 0x%x\r\n", statusRegValue1);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_1 ATCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_1 ATCM1 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B0TCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_B0TCM1,\
								 SDL_R5F0B0TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B0TCM1 = 0x%x\r\n", statusRegValue1);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_1 B0TCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_1 B0TCM1 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B1TCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_B1TCM1,\
								 SDL_R5F0B1TCM1_MASK);

		/* Wait until ESM interrupt happens */while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B1TCM1 = 0x%x\r\n", statusRegValue1);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS0_1 B1TCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS0_1 B1TCM1 Parity : Completed\r\n");
		}
	}

/* R5FSS1_0 */

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY:R5FSS1_0\r\n");
		DebugP_log("\r\nTCM PARITY : ATCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_ATCM0,\
								 SDL_R5F1ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for ATCM0 = 0x%x\r\n", statusRegValue2);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_0 ATCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_0 ATCM0 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY:B0TCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_B0TCM0,\
								 SDL_R5F1B0TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B0TCM0 = 0x%x\r\n", statusRegValue2);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_0 B0TCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_0 B0TCM0 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B1TCM0 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_B1TCM0,\
								 SDL_R5F1B1TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B1TCM0 = 0x%x\r\n", statusRegValue2);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_0 B1TCM0 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_0 B1TCM0 Parity : Completed\r\n");
		}
	}

/* R5FSS1_1 */

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY:R5FSS1_1\r\n");
		DebugP_log("\r\nTCM PARITY:ATCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_ATCM1,\
								 SDL_R5F1ATCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for ATCM1 = 0x%x\r\n", statusRegValue3);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_1 ATCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_1 ATCM1 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B0TCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_B0TCM1,\
								 SDL_R5F1B0TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B0TCM1 = 0x%x\r\n", statusRegValue3);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_1 B0TCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_1 B0TCM1 Parity : Completed\r\n");
		}
	}

	if (retVal == 0)
    {
		DebugP_log("\r\nTCM PARITY : B1TCM1 Started\r\n");
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_B1TCM1,\
								 SDL_R5F1B1TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(esmError !=true);
		esmError = false;
		DebugP_log("\r\nTCM Parity Status for B1TCM1 = 0x%x\r\n", statusRegValue3);
		if (result != SDL_PASS)
		{
			retVal = -1;
			DebugP_log("\r\nR5FSS1_1 B1TCM1 Parity : Failed\r\n");
		}
		else
        {
			retVal = 0;
			DebugP_log("\r\nR5FSS1_1 B1TCM1 Parity : Completed\r\n");
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

/* Parity Function module test */
int32_t Parity_funcTest(void)
{
    int32_t testResult = 0;

    /*Initializing the DPL*/
    sdlApp_dplInit();

    /*Initializing required modules*/
    testResult = Parity_Example_init();

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\nParity Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    /*Execute ECC sdl function test*/
    testResult = Parity_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
