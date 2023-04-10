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
 *  \file     dss_l2_edc.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Detecting and Correcting (EDC) Module application.
 *
 *  \details  EDC Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include "edc_main.h"
#include <sdl/dpl/sdl_dpl.h>

#pragma CODE_SECTION(EDC_dummyFunction, ".func");
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_EDC_SEC                       			(0x01u)
#define SDL_EDC_DED                       			(0x03u)

#define SDL_DSS_L2_ORIGIN                           (0x00800000u)
#define SDL_DSS_INTH_INT_ID_IDMAINT1                (14)

#define SDL_DSS_ICFGF_L1PCFG                        (0x01840020U)
#define SDL_DSS_ICFGF_L1DCFG                        (0x01840040U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern volatile bool esmError;
extern volatile bool idmaTransferComplete;

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);
extern int32_t SDL_IDMA1_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg);

/* Event BitMap for EDC ESM callback for DSS */
SDL_ESM_NotifyParams EDC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for EDC ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_SEC_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
      {
           /* Event BitMap for EDC ESM callback for DSS Double bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_DED_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
	
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static int32_t EDC_sdlFuncTest(uint32_t injectType);
int32_t EDC_dummyFunction(void);

/*********************************************************************
* @fn      EDC_dummyFunction
*
* @param   None
*
* @return  dummy operation value
**********************************************************************/
int32_t EDC_dummyFunction(void)
{
    int32_t a = 0;
    int32_t b = 4;
    int32_t i;
    for (i = 0; i <= 3; i++)
    {
        b = b + 7;
        a = a + b;
    }
    return a;
}/* End of EDC_dummyFunction() */

/*********************************************************************
* @fn      EDC_Example_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t EDC_Example_init (void)
{
    int32_t retValue=0, counter = 0;
    SDL_ErrType_t result;
    if (retValue == 0) {
		for(counter = 0; counter < SDL_ESM_MAX_DSS_EXAMPLE_AGGR; counter++)
		{
			result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &EDC_TestparamsDSS[counter],NULL,NULL);
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
}/* End of EDC_Example_init() */

/*********************************************************************
 * @fn      EDC_sdlFuncTest
 *
 * @brief   Execute EDC sdl function test
 *
 * @param   injectType: Single or Double bit
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t EDC_sdlFuncTest(uint32_t injectType)
{
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_EDC_CfgHwiPHandle = NULL;
    volatile uint32_t rd_data;

    DebugP_log("\r\nEDC Safety Example tests: starting\r\n");

    if (retVal == 0)
    {
        /* Configuring the ESM */
        intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;

        /* Configuring the IDMA1 DED interrupt */
        intrParams.intNum = SDL_DSS_INTH_INT_ID_IDMAINT1;
        intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_IDMA1_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the IDMA 1 interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_INTH_INT_ID_IDMAINT1);

        DebugP_log("\r\nInitiliazed and Enabled IDMA1 Interrupt\r\n");

        DebugP_log("\r\nEnable the Error Detect logic...\r\n");
        retVal = SDL_ECC_dss_l2_edc_CMD_EN();
        if (retVal == SDL_PASS)
        {
            SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L2_ORIGIN);
            DebugP_log("\r\nWaiting for IDMA1 transfer Interrupt\r\n");
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    retVal = SDL_EFAIL;
                    DebugP_log("\r\nIDMA1 interrupt is not occurred.... Example is failed!!\r\n");
                    break;
                }
            } while (idmaTransferComplete == false);

            DebugP_log("\r\nIDMA1 transfer is done and got interrupt !!\r\n");

            EDC_dummyFunction();

            DebugP_log("\r\nSuspend the Error Detect logic...\r\n");
            retVal = SDL_ECC_dss_l2_CMD_SUSP();
            if (retVal == SDL_PASS)
            {
                if(injectType == SDL_EDC_SEC)
                {
                    DebugP_log("\r\nToggle a single bit in the Dummy function\r\n");
                    rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
                    rd_data = rd_data ^ SDL_EDC_SEC;
                    SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
                }
                else if(injectType == SDL_EDC_DED)
                {
                    DebugP_log("\r\nToggle double bit in the Dummy function\r\n");
                    rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
                    rd_data = rd_data ^ SDL_EDC_DED;
                    SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
                }
                else{
                    /* No EDC inject */
                }

                DebugP_log("\r\nEnable the Error Detect logic...\r\n");
                retVal = SDL_ECC_dss_l2_edc_CMD_EN();
                if (retVal == SDL_PASS)
                {
                    DebugP_log("\r\nCall dummy function\r\n");
                    EDC_dummyFunction();

                    DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
                    do
                    {
                        timeOutCnt += 10;
                        if (timeOutCnt > maxTimeOutMilliSeconds)
                        {
                            retVal = SDL_EFAIL;
                            DebugP_log("\r\nESM interrupt is not occurred.... Example is failed!!\r\n");
                            break;
                        }
                    } while (esmError == false);
                    if(esmError == true)
                    {
                        DebugP_log("\r\nESM Interrupt has occurred!!\r\n");
                        if(injectType == SDL_EDC_SEC)
                        {
                            DebugP_log("\r\nSEC test has passed!!\r\n");
                        }
                        else
                        {
                            DebugP_log("\r\nDED test has passed!!\r\n");
                        }
                        esmError = false;
                        retVal = SDL_PASS;
                    }
                }
                else
                {
                    DebugP_log("\r\nEnable Error Detect logic is failed after IDMA transfer and Error injection...\r\n");
                    retVal = SDL_EFAIL;
                }
            }
            else
            {
                DebugP_log("\r\nSuspend Error Detect logic is failed...\r\n");
                retVal = SDL_EFAIL;
            }
        }
        else
        {
            DebugP_log("\r\nEnable Error Detect logic is failed...\r\n");
            retVal = SDL_EFAIL;
        }
    }
    return(retVal);
}/* End of EDC_sdlFuncTest() */

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }

    return ret;
}/* End of sdlApp_dplInit() */

/* EDC Function module test */
int32_t EDC_funcTest(void)
{
    int32_t testResult = SDL_PASS;
	 /*Initializing the DPL*/
    sdlApp_dplInit();

    /*L1PCFG is set to 0*/
    SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00);
    /*L1DCFG is set to 0*/
    SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00);

    testResult = EDC_Example_init();

    if (testResult != 0)
    {
        DebugP_log("\r\nEDC Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    testResult = EDC_sdlFuncTest(SDL_EDC_SEC);
    if (testResult != 0)
    {
        DebugP_log("\r\nEDC SEC Safety Example failed\r\n");
        return SDL_EFAIL;
    }

    /*L1PCFG is set to 0*/
    SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00);
    /*L1DCFG is set to 0*/
    SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00);
    testResult = EDC_sdlFuncTest(SDL_EDC_DED);
    if (testResult != 0)
    {
        DebugP_log("\r\nEDC DED Safety Example failed\r\n");
        return SDL_EFAIL;
    }

    return (testResult);
}/* End of EDC_funcTest() */

/* Nothing past this point */
