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
 *  \file     ecc_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Correcting Code (ECC) Module application.
 *
 *  \details  ECC Safety Example module tests
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
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>

#if defined(SOC_AM273X)
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/am273x/sdlr_dss_ecc_agg.h>
#endif
#if defined(SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#include <sdl/include/awr294x/sdlr_intr_esm_dss.h>
#include <sdl/include/awr294x/sdlr_dss_ecc_agg.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay for 1us*/
#define DELAY 1

#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)


#define SDL_DSS_MAX_MEM_SECTIONS                    (1u)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern uint32_t testCounter;

static SDL_ECC_MemSubType ECC_Test_DSSsubMemTypeList[SDL_DSS_MAX_MEM_SECTIONS] =
{
     SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_DSSECCInitConfig =
{
    .numRams = SDL_DSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_DSSsubMemTypeList[0]),
    /**< Sub type list  */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);

/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams ECC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
     {
           /* Event BitMap for ECC ESM callback for DSS Single bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_SERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
      {
           /* Event BitMap for ECC ESM callback for DSS Double bit*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_UERR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ESM_applicationCallbackFunction,
      },
	
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      ECC_Example_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Example_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

             retValue = -1;
         } else {
             DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
         }
    }
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_DSS_ECC_AGG, &ECC_Test_DSSECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing DSS ECC AGGR: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: DSS ECC AGGR initialization is completed \n");
        }
    }
    return retValue;
}

/*********************************************************************
 * @fn      ECC_Test_run_DSS_L3RAMA_1BitInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_L3RAMA_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMA Single bit error inject: test starting");

	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_L3RAMA_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_L3RAMA_2BitInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_L3RAMA_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMA Double bit error inject: starting");

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\n DSS L3RAMA Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_L3RAMA_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;

    DebugP_log("\n\n ECC Safety Example tests: starting");

    if (retVal == 0)
    {
		result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        if (result != SDL_PASS) {
           /* print error and quit */
            DebugP_log("ESM_Test_init: Error initializing DSS ESM: result = %d\n", result);
        }
        else
        {
            DebugP_log("\nESM_Test_init: Init DSS ESM complete \n");
        }
        result = ECC_Test_run_DSS_L3RAMA_1BitInjectTest();
        if (result == SDL_PASS)
        {
            DebugP_log("\n\n Waiting for ESM Interrupt \n\n");
            do
            {
                timeOutCnt += 1;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if(result == SDL_PASS){
            DebugP_log("\n\nUC-1: Injected 1-bit error and got ESM Interrupt \n\n");
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\nESM_ECC_Example_run: UC-1 has failed...");
            /* UC-1 Low priority R5F interrupt */
        }
    }
    if (retVal == 0) {
		result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        if (result != SDL_PASS) {
           /* print error and quit */
            DebugP_log("ESM_Test_init: Error initializing DSS ESM: result = %d\n", result);


        } 
        else
        {
            DebugP_log("\nESM_Test_init: Init DSS ESM complete \n");
        }
        result = ECC_Test_run_DSS_L3RAMA_2BitInjectTest();
        if (result == SDL_PASS)
        {
            DebugP_log("\n\n Waiting for ESM Interrupt \n\n");
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if(result == SDL_PASS){
            DebugP_log("\n\n UC-2: Injected 2-bit error and got ESM Interrupt \n\n");
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\n ESM_ECC_Example_run: UC-2 has failed....");
            /* UC-2 High priority R5F interrupt */
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
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}
/* ECC Function module test */
int32_t ECC_funcTest(void)
{
    int32_t testResult;
    sdlApp_dplInit();
    testResult = ECC_Example_init();

    if (testResult != 0)
    {
        DebugP_log("\n\n ECC Safety Example tests: unsuccessful");
        return SDL_EFAIL;
    }

    testResult = ECC_sdlFuncTest();


    return (testResult);
}

/* Nothing past this point */
