/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file     dss_l2_parity.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Parity Module application.
 *
 *  \details  Parity Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_exception.h>
#include "parity_main.h"
#include <sdl/dpl/sdl_dpl.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(1u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_INJECT_PARITY                           (0x01u)

#define SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1     (0x06020074u)
#define SDL_INITIAL_VALUE                           (0x11u)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);

/* Event BitMap for ESM callback for DSS */
SDL_ESM_NotifyParams Parity_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for Parity ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_PARITY_ERR_VB0_EVEN,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },	
};
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
    int32_t retValue=0, counter = 0;
    SDL_ErrType_t result;
    if (retValue == 0) {
		for(counter = 0; counter < SDL_ESM_MAX_DSS_EXAMPLE_AGGR; counter++)
		{
			result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &Parity_TestparamsDSS[counter],NULL,NULL);
			if (result != SDL_PASS) {
				retValue = -1;
			   /* print error and quit */
				DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
			}
		}
        if(retValue == 0)
        {
            DebugP_log("\r\nESM_Test_init: Init DSS ESM complete\r\n");
        }
    }
    return retValue;
}

/*********************************************************************
 * @fn      parity_sdlFuncTest
 *
 * @brief   Execute Parity sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t Parity_sdlFuncTest(void)
{
    int32_t result = 0;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0u;
    uint32_t injectErrAdd = 0u;

    DebugP_log("\r\nDSS L2 parity Safety Example tests: starting\r\n");

    if (retVal == 0)
    {
        /*ESM init*/
        Parity_Example_init();

        /*
         *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
         *Disable register field
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);
        /*
         * Enable the parity
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ENABLE);
        DebugP_log("\r\nDSS L2 parity is enabled\r\n");

        /*
         *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
         *Disable register field
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);
        /*dss l2 parity init*/
        SDL_ECC_dss_l2_parity_init();
        DebugP_log("\r\nDSS L2 parity initialization is completed\r\n");

        /* On a parity error from a particular bank, reading the register
         * DSS_CTRL.DSS_DSP_L2RAM_PARITY_ERR_STATUS_VBx gives the address
         * location
         */
        injectErrAdd = SDL_REG32_RD(SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1);

        esmError = false;

        /*DSS L2 parity error inject*/
        SDL_ECC_dss_l2_parity_errorInject(SDL_INJECT_PARITY, injectErrAdd, SDL_INITIAL_VALUE);

        DebugP_log("\r\nDSS L2 parity error injected\r\n");

        DebugP_log("\r\nWaiting for ESM Interrupt \r\n");
		do
		{
			timeOutCnt += 1;
			if (timeOutCnt > maxTimeOutMilliSeconds)
			{
				result = SDL_EFAIL;
				break;
			}
		} while (esmError == false);
		
        if(result == SDL_PASS){
			DebugP_log("\r\ncleared DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE\r\n");
            DebugP_log("\r\nUC-1: Parity error is injected 1-bit error and got ESM Interrupt \r\n");
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nESM_Parity_Example_run: UC-1 has failed...\r\n");
            /* UC-1 Low priority interrupt */
        }
    }

    return retVal;
}

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
    int32_t testResult;
    sdlApp_dplInit();

    testResult = Parity_sdlFuncTest();


    return (testResult);
}

/* Nothing past this point */
