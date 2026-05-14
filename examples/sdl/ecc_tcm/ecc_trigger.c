/*
 *   Copyright (c) Texas Instruments Incorporated 2023-2025
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
 *  \file    ecc_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Correcting Code (ECC) Module application.
 *
 *  \details  ECC TCM Safety Example
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <sdl/sdl_esm.h>
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/include/am64x_am243x/sdlr_soc_ecc_aggr.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* Defines */
#define ECC_AGGR_MAX_MEM_SECTIONS (6u)
/* Change below two macros to test different TCM's */

#if defined(SOC_AM64X) || defined (SOC_AM243X)
#define SDL_TCM_ERROR__INJECT_ADDRESS CSL_R5FSS0_BTCM_BASE
#define SDL_TCM_AGGR_TO_TEST  SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR
#define SDL_TCM_RAMID_TO_TEST SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B0TCM0_BANK0_RAM_ID
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

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

    // SDLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE0_ECC_CORRECTED_LEVEL_0      30
    // SDLR_ESM0_ESM_LVL_EVENT_R5FSS0_CORE0_ECC_UNCORRECTED_LEVEL_0    91
    // SDLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_DE_TO_ESM_0_0        40
    // SDLR_ESM0_ESM_LVL_EVENT_R5FSS0_COMMON0_ECC_SE_TO_ESM_0_0        42        

#if defined(SOC_AM64X) || defined(SOC_AM243X)
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    // .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = { 0x40000000u, 0x00000500u, 0x08000000u,0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u,0x00000000u,
                    },
     /**< All events enable: except clkstop events for unused clocks */
    .priorityBitmap = { 0x40000000u, 0x00000500u, 0x08000000u,0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u,0x00000000u,
                      },
    /**< All events high priority: except clkstop events for unused clocks */
    .errorpinBitmap = { 0x00000000u, 0x00000000u, 0x00000000u,0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u,0x00000000u,
                      },
    /**< Error pin assert not enabled */
};
#endif

// #if defined(SOC_AM62AX) || defined (SOC_AM62DX) || defined (SOC_AM62PX)
// static SDL_ECC_MemSubType ECC_Test_AGGR_subMemTypeList[ECC_AGGR_MAX_MEM_SECTIONS] =
// {
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_ATCM0_BANK0_RAM_ID,
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_ATCM0_BANK1_RAM_ID,
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_B0TCM0_BANK0_RAM_ID,
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_B0TCM0_BANK1_RAM_ID,
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_B1TCM0_BANK0_RAM_ID,
//     SDL_MCU_R5FSS0_PULSAR_ULS_CPU0_ECC_AGGR_PULSAR_ULS_B1TCM0_BANK1_RAM_ID,
// };
// #endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
static SDL_ECC_MemSubType ECC_Test_AGGR_subMemTypeList[ECC_AGGR_MAX_MEM_SECTIONS] =
{
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK1_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_ATCM1_BANK0_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_ATCM1_BANK1_RAM_ID,
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B0TCM0_BANK0_RAM_ID,
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B0TCM0_BANK1_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_B0TCM1_BANK0_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_B0TCM1_BANK1_RAM_ID,
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B1TCM0_BANK0_RAM_ID,
    SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B1TCM0_BANK1_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_B1TCM1_BANK0_RAM_ID,
    // SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR_PULSAR_LITE_B1TCM1_BANK1_RAM_ID,
};
#endif

static SDL_ECC_InitConfig_t ECC_Test_AGGR0A0ECCInitConfig =
{
    .numRams = ECC_AGGR_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_AGGR_subMemTypeList[0]),
    /**< Sub type list  */
};

static uint32_t arg;

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

int32_t ECC_Test_EsmInitHandlerInit(SDL_ESM_Inst esmInstType);
int32_t ECC_Example_init (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */




/*********************************************************************
* @fn      ECC_Example_init
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/

int32_t ECC_Example_init (void)
{
    int32_t retValue = SDL_APP_TEST_PASS;
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;

    /* Initialise exception handler
     * Double bit error in TCM causes abort. Initialize custom abort handlers to make sure code is not stuck in infinite loop. */
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);

    /* Initialize MAIN ESM module */
    #if defined(SOC_AM64X) || defined(SOC_AM243X)
    DebugP_log("\r\nECC_Example_init: Initializing MAIN ESM.\r\n");
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
    if (result != SDL_APP_TEST_PASS)
    {
        /* print error and quit */
        DebugP_log("\r\nECC_Example_init: Error initializing MAIN ESM: result = %d\r\n", result);

        retValue = SDL_APP_TEST_FAILED;
    }
    else
    {
        DebugP_log("\r\nECC_Example_init: Init MAIN ESM complete \r\n");
    }
    #endif
    if (retValue == SDL_APP_TEST_PASS)
    {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_TCM_AGGR_TO_TEST, &ECC_Test_AGGR0A0ECCInitConfig);

        if (result != SDL_APP_TEST_PASS)
        {
            /* print error and quit */
             DebugP_log("\r\nError initializinECC Aggregator : result = %d\r\n", result);

             retValue = SDL_APP_TEST_FAILED;
        }
        else
        {
            DebugP_log("\r\n\n AGGR ECC Init complete \r\n");
        }
    }

    return retValue;
}

/*********************************************************************
 * @fn    ECC_Test_runECC1BitB0TCM0Bank0_InjectTest
 *
 * @brief   Execute ECC Double bit error example test on ECC aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC1BitB0TCM0Bank0_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal = SDL_APP_TEST_PASS;
    volatile uint32_t testLocationValue;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\nB0TCM0 Bank0 Single bit error inject test: starting\r\n");

    /* Run one shot test for BTCM0 1 bit error
     * Note the address is relative to start of ram
     * Note that the DM swaps the ATCM and BTCM addresses. ATCM starts at 0x41010000 and BTCM starts at 0x0 */
    injectErrorConfig.pErrMem = (uint32_t *)SDL_TCM_ERROR__INJECT_ADDRESS;
    injectErrorConfig.flipBitMask = 0x5;
    injectErrorConfig.chkGrp = 0x1;

    result = SDL_ECC_injectError(SDL_TCM_AGGR_TO_TEST,
                                 SDL_TCM_RAMID_TO_TEST,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\nB0TCM0 Bank0 Single bit error inject test: at pErrMem 0x%p: fixed location once test failed", injectErrorConfig.pErrMem);
        retVal = SDL_APP_TEST_FAILED;
    }
    else
    {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        /* To avoid compiler warning on unused variable */
        testLocationValue = testLocationValue;
        DebugP_log("\r\nB0TCM0 Bank0 Single bit error inject test: at pErrMem 0x%p: fixed location once test complete", injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of runECC2BitAGGR0_InjectTest() */

int32_t ECC_Test_runECC2BitB0TCM0Bank0_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    volatile uint32_t testLocationValue;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n B0TCM0 Bank0 Double bit error inject test: starting");

    /* Run one shot test for BTCM0 2 bit error
     * Note the address is relative to start of ram
     * Note that the DM swaps the ATCM and BTCM addresses. ATCM starts at 0x41010000 and BTCM starts at 0x0 */
    injectErrorConfig.pErrMem = (uint32_t *)SDL_TCM_ERROR__INJECT_ADDRESS;
    injectErrorConfig.flipBitMask = 0x101;
    injectErrorConfig.chkGrp = 0x1;

    result = SDL_ECC_injectError(SDL_TCM_AGGR_TO_TEST,
                                 SDL_TCM_RAMID_TO_TEST,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS)
    {
        DebugP_log("\r\n B0TCM0 Bank0 Double bit error inject test: at pErrMem 0x%p: fixed location once test failed", injectErrorConfig.pErrMem);
        retVal = -1;
    }
    else
    {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        /* To avoid compiler warning on unused variable */
        testLocationValue = testLocationValue;
        DebugP_log("\r\n B0TCM0 Bank0 Double bit error inject test: pErrMem 0x%p fixed location once test complete", injectErrorConfig.pErrMem);
    }

    return retVal;
}

static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = SDL_APP_TEST_PASS;
    uint32_t maxTimeOut = 1000000000;
    uint32_t timeOutCnt = 0;

    DebugP_log("\r\nECC Safety Example tests: starting\r\n");

    result = ECC_Test_runECC2BitB0TCM0Bank0_InjectTest();

    if (result == SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
        do
        {
            timeOutCnt += 1;
            if (timeOutCnt > maxTimeOut)
            {
                result = SDL_EFAIL;
                break;
            }
        } while (esmError == false);
    }
    if (result == SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\nUC-1: Got High priority ESM Interrupt \r\n");
        esmError = false;
    }

    if (result != SDL_APP_TEST_PASS)
    {
        retVal = SDL_APP_TEST_FAILED;
        DebugP_log("\r\nESM_ECC_Example_run: UC-1 has failed...\r\n");
        /* UC-1 High priority interrupt */
    }


    if (retVal == SDL_APP_TEST_PASS)
    {
        result = ECC_Test_runECC1BitB0TCM0Bank0_InjectTest();
        if (result == SDL_APP_TEST_PASS)
        {
            DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOut)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if (result == SDL_APP_TEST_PASS){
            DebugP_log("\r\nUC-2: Got Low priority ESM Interrupt\r\n");
            esmError = false;
        }

        if (result != SDL_APP_TEST_PASS)
        {
            retVal = SDL_APP_TEST_FAILED;
            DebugP_log("\r\nESM_ECC_Example_run: UC-2 has failed....\r\n");
            /* UC-2 Low priority interrupt */
        }
    }

    if ( retVal == SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\nECC Safety Example tests: success\r\n");
    }
    else
    {
        DebugP_log("\r\nECC Safety Example tests: failed\r\n");
    }

    return retVal;
}

/* ECC Function module test */
int32_t ECC_funcTest(void)
{
    int32_t testResult;

    testResult = ECC_Example_init();

    if (testResult != SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\nECC Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
