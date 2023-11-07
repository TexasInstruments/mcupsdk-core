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
/* value to write into param register */
#define PARAM_REG_VALUE  0x7u
/* Enable testmode and parity in TPCC0_PARITY_CTRL register */
#define PARITY_ENABLE    0x11u
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint32_t arg;
SDL_ESM_config Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events enable: except clkstop events for unused clocks
    *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
    /**< All events high priority: except clkstop events for unused clocks
    *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x80000000u, 0x00000010u, 0x00000000u,
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
	int32_t i=0;

	DebugP_log("\r\nTPCC PARITY Example : Started\r\n");

	if (retVal == 0)
    {
		DebugP_log("\r\nTPCC PARITY : TPCC0 Started\r\n");
        retVal = SDL_ECC_tpccParity(SDL_TPCC0, \
								 PARITY_ENABLE, SDL_PARAM_REG_1, \
								 PARAM_REG_VALUE);
		DebugP_log("\r\nParam Register = %x\r\n", retVal);

		/* wait for delay */
		for (i =0 ; i < 200 ; i = i + 1 );

		/* Wait until ESM interrupt happens */
		while(esmError !=true);

		if(esmError == true)
		{
			DebugP_log("\r\nTPCC PARITY : TPCC0 Completed\r\n");
			esmError = false;
		}
		else
        {
			result = SDL_EFAIL;
		}
	}

    return result;
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
