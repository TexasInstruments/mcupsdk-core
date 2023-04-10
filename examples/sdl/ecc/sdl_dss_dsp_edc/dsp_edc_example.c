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
 *  \file     dsp_edc_example.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Detection and correction (EDC) Module application.
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

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_DSS_ECC_NC_ITERRUPT_ID					(111)
#define SDL_DSS_ECC_C_ITERRUPT_ID					(112)

#define SDL_ENABLE_L1D_DATA_MASK_FLG                (0x01u)
#define SDL_ENABLE_L1D_TAG_MASK_FLG                 (0x02u)
#define SDL_ENABLE_L2_TAG_MASK_FLG                  (0x04u)
#define SDL_ENABLE_L2_SNOP_MASK_FLG                 (0x08u)
#define SDL_ENABLE_L2_MPPA_MASK_FLG                 (0x10u)
#define SDL_ENABLE_L2_LRU_MASK_FLG                  (0x20u)
#define SDL_ENABLE_L1P_TAG_MASK_FLG                 (0x40u)

/* To test above memories one by one then assign any of the above macro to SDL_MEMORY_ENABLE macro*/
#define SDL_MEMORY_ENABLE							SDL_ENABLE_L2_SNOP_MASK_FLG

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);

extern int32_t SDL_ECC_DED_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

extern int32_t SDL_ECC_SEC_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

/* Event BitMap for EDC ESM callback for DSS */
SDL_ESM_NotifyParams EDC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for EDC ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_SEC_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
      {
           /* Event BitMap for EDC ESM callback for DSS Double bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_DED_ERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
	
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

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
        /* Initialize ESM */
        for(counter = 0; counter < SDL_ESM_MAX_DSS_EXAMPLE_AGGR; counter++)
        {
            result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &EDC_TestparamsDSS[counter],NULL,NULL);
            if (result != SDL_PASS) {
               /* print error and quit */
                DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
            }
        }
        if (result == SDL_PASS)
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
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/

static int32_t EDC_sdlFuncTest(void)
{
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    uint32_t readValue = 0;
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_EDC_CfgHwiPHandle = NULL;

    DebugP_log("\r\nEDC Safety Example tests: starting\r\n");

    if (retVal == 0)
    {
        /* Configuring the ESM */
		intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;
		
		/* Configuring the ECC DED interrupt */
		intrParams.intNum = SDL_DSS_ECC_NC_ITERRUPT_ID;
		intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_DED_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the DED interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_ECC_NC_ITERRUPT_ID);

        DebugP_log("\r\nInitiliazed and Enabled ECC DED Interrupt\r\n");
		
        /* Configuring the ECC SEC interrupt */
        intrParams.intNum = SDL_DSS_ECC_C_ITERRUPT_ID;
        intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_SEC_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the SEC interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_ECC_C_ITERRUPT_ID);

        DebugP_log("\r\nInitiliazed and Enabled ECC SEC Interrupt\r\n");

        /*Write to DSP_ICFG__EDCINTMASK and DSP_ICFG__EDCINTFLG registers
         * to enable and propagate an DSS DSP Memories
         */
        DebugP_log("\r\nEnabling and Propagating the memory\r\n");
        SDL_ECC_DSP_Aggregated_EDC_Errors(SDL_MEMORY_ENABLE);


        DebugP_log("\r\nWaiting for DSS Interrupt\r\n");
        do
        {
            timeOutCnt += 10;
            if (timeOutCnt > maxTimeOutMilliSeconds)
            {
                retVal = SDL_EFAIL;
                DebugP_log("\r\nDSS interrupt is not occurred.... Example is failed!!\r\n");
                break;
            }
        } while (esmError == false);

        readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTMASK);
        if(readValue == 0){
            readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTFLG);
            if(readValue == 0)
            {
				DebugP_log("\r\nDisabled the MASK and FLG registers\r\n");
				DebugP_log("\r\nUC-1: DSP EDC Error tested and got all Interrupts \r\n");
                retVal = SDL_PASS;
            }
            else
            {
				DebugP_log("\r\nEDC FLG register is not equal to 0...... Example is failed!!\r\n");
                retVal = SDL_EFAIL;
            }
        }
        else
        {
			DebugP_log("\r\nEDC MASK register is not equal to 0...... Example is failed!!\r\n");
            retVal = SDL_EFAIL;
        }
    }

    return retVal;
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
}
/* EDC Function module test */
int32_t EDC_funcTest(void)
{
    int32_t testResult;
    sdlApp_dplInit();
    testResult = EDC_Example_init();

    if (testResult != 0)
    {
        DebugP_log("\r\nEDC Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    testResult = EDC_sdlFuncTest();


    return (testResult);
}/* End of EDC_funcTest() */

/* Nothing past this point */
