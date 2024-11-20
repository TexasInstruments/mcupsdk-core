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
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/**
*	\brief  Developer can enable or disable below macros for the testing off
*           BTCM different banks
* 			Only one macro should be enabled at one time
**/
#define SDL_B0TCM0_BANK0							(0U)
#define	SDL_B0TCM0_BANK1							(0U)
#define	SDL_B1TCM0_BANK0							(0U)
#define	SDL_B1TCM0_BANK1							(1U)

#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#if SDL_B0TCM0_BANK0
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00080000u) /* R5F B0TCM Bank 0 RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID
#endif

#if SDL_B0TCM0_BANK1
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00080004u) /* R5F B0TCM Bank 1 RAM address */
#if defined (R5F0_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID
#elif defined (R5F1_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS1_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID
#endif
#endif

#if SDL_B1TCM0_BANK0
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x00080008u) /* R5F B1TCM Bank 0 RAM address */
#if defined (R5F0_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID
#elif defined (R5F1_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS1_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID
#endif
#endif

#if SDL_B1TCM0_BANK1
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x0008000cu) /* R5F B1TCM Bank 1 RAM address */
#if defined (R5F0_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID
#elif defined (R5F1_INPUTS)
#define SDL_EXAMPLE_ECC_AGGR                        SDL_R5FSS1_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID
#endif
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern uint32_t testCounter;

/* This is the list of exception handle and the parameters */
const SDL_R5ExptnHandlers ECC_Test_R5ExptnHandlers =
{
    .udefExptnHandler = &SDL_EXCEPTION_undefInstructionExptnHandler,
    .swiExptnHandler = &SDL_EXCEPTION_swIntrExptnHandler,
    .pabtExptnHandler = &SDL_EXCEPTION_prefetchAbortExptnHandler,
    .dabtExptnHandler = &SDL_EXCEPTION_dataAbortExptnHandler,
    .irqExptnHandler = &SDL_EXCEPTION_irqExptnHandler,
    .fiqExptnHandler = &SDL_EXCEPTION_fiqExptnHandler,
    .udefExptnHandlerArgs = ((void *)0u),
    .swiExptnHandlerArgs = ((void *)0u),
    .pabtExptnHandlerArgs = ((void *)0u),
    .dabtExptnHandlerArgs = ((void *)0u),
    .irqExptnHandlerArgs = ((void *)0u),
};

void ECC_Test_undefInstructionExptnCallback(void)
{
    printf("\r\nUndefined Instruction exception\r\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    printf("\r\nSoftware interrupt exception\r\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    printf("\r\nPrefetch Abort exception\r\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    printf("\r\nData Abort exception\r\n");
}
void ECC_Test_irqExptnCallback(void)
{
    printf("\r\nIrq exception\r\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    printf("\r\nFiq exception\r\n");
}

void ECC_Test_exceptionInit(void)
{

    SDL_EXCEPTION_CallbackFunctions_t exceptionCallbackFunctions =
            {
                .udefExptnCallback = ECC_Test_undefInstructionExptnCallback,
                .swiExptnCallback = ECC_Test_swIntrExptnCallback,
                .pabtExptnCallback = ECC_Test_prefetchAbortExptnCallback,
                .dabtExptnCallback = ECC_Test_dataAbortExptnCallback,
                .irqExptnCallback = ECC_Test_irqExptnCallback,
                .fiqExptnCallback = ECC_Test_fiqExptnCallback,
            };

    /* Initialize SDL exception handler */
    SDL_EXCEPTION_init(&exceptionCallbackFunctions);
    /* Register SDL exception handler */
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);

    return;
}

static uint32_t arg;

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x01818000u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events enable: except clkstop events for unused clocks
     *   and PCIE events */
    /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x00000000u, 0x01010000u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x01818000u, 0x00000000u, 0x00000000u,
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

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
    SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ECC_Example_init function */
int32_t ECC_Example_init (void);

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
    void *ptr = (void *)&arg;
    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

    if (retValue == 0) {
            /* Initialize ECC Memory */
        result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing Memory of R5FSS0 CORE0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete \r\n");
        }
    }
    if (retValue == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);


            retValue = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }

    }
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing R5FSS0 CORE0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 ECC initialization is completed \r\n");
        }
    }
    return retValue;
}

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_BTCM_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 BTCM 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_BTCM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 BTCM Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for R5FSS0 CORE0 BTCM 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nR5FSS0 CORE0 BTCM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_BTCM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_BTCM_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 BTCM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_BTCM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 BTCM Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 BTCM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 BTCM Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_BTCM_2BitInjectTest() */

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

    if (retVal == 0)
    {
        result = ECC_Test_run_R5FSS0_CORE0_BTCM_2BitInjectTest();
        if (result == SDL_PASS)
        {
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
        }
        if(result == SDL_PASS){
            DebugP_log("\r\nUC-1: Injected 2-bit error and got ESM Interrupt\r\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nESM_ECC_Example_run: UC-1 has failed...\r\n");
            /* UC-1 Low priority R5F interrupt */
        }
    }

	if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_BTCM_1BitInjectTest();
        if (result == SDL_PASS)
        {
            DebugP_log("\r\nWaiting for ESM Interrupt \r\n");
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
            DebugP_log("\r\nUC-2: Injected 1-bit error and got ESM Interrupt\r\n");
			/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nESM_ECC_Example_run: UC-2 has failed....\r\n");
            /* UC-2 High priority R5F interrupt */
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
int32_t ECC_funcTest(void)
{
    int32_t testResult = 0;

    /*Initializing the DPL*/
    sdlApp_dplInit();

#if (SDL_B0TCM0_BANK0) || (SDL_B0TCM0_BANK1)
    /*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB0TCM();
#endif
#if (SDL_B1TCM0_BANK0) || (SDL_B1TCM0_BANK1)
	/*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB1TCM();
#endif

    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    /*Initializing required modules*/
    testResult = ECC_Example_init();

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\nECC Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    /*Execute ECC sdl function test*/
    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
