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
 *  \file    ecc_trigger.c
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
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am64x_am243x/sdlr_soc_ecc_aggr.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* delay for 1us*/
#define DELAY 1
#if defined (M4F_CORE)
/* Initialization structure for MCU ESM instance */
static SDL_ESM_config ECC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xffffffffu, 0x033fffffu, 0x7fffffffu, 0xffffffe7u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
    .priorityBitmap = {0xffffffffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    .errorpinBitmap = {0xffffffffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
};

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xfffffb0fu, 0xf7c0000eu, 0xf7c0000eu, 0x00008f80u,
                     0x00001800u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0xfffffb0fu, 0xf7c0000eu, 0xf7c0000eu, 0x00008f80u,
                       0x00001800u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xfffffb0fu, 0xf7c0000eu, 0xf7c0000eu, 0x00008f80u,
                       0x00001800u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#endif

#if defined (R5F_CORE)
/* Initialization structure for MCU ESM instance */
static SDL_ESM_config ECC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
    .priorityBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    .errorpinBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
};

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xfffffb0fu, 0xffc000efu, 0xffbffd8f, 0x00008f80u,
                     0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0xfffffb0fu, 0xffc000efu, 0xffbffd8f, 0x00008f80u,
                       0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xfffffb0fu, 0xffc000efu, 0xffbffd8f, 0x00008f80u,
                       0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

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
    DebugP_log("\r\nUndefined Instruction exception\r\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    DebugP_log("\r\nSoftware interrupt exception\r\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    DebugP_log("\r\nPrefetch Abort exception\r\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    DebugP_log("\r\nData Abort exception\r\n");
}
void ECC_Test_irqExptnCallback(void)
{
    DebugP_log("\r\nIrq exception\r\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    DebugP_log("\r\nFiq exception\r\n");
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
#endif

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Defines */
#define MAIN_AGGR1_AGGR1_MAX_MEM_SECTIONS (2u)


int32_t ECC_Test_EsmInitHandlerInit(SDL_ESM_Inst esmInstType);
int32_t ECC_Example_init (void);

static SDL_ECC_MemSubType ECC_Test_AGGR1_A0subMemTypeList[MAIN_AGGR1_AGGR1_MAX_MEM_SECTIONS] =
{
    SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_IK3VTM_N16FFC_MAIN_0_VBUSP_P2P_BRIDGE_IK3VTM_N16FFC_MAIN_0_VBUSP_BRIDGE_BUSECC_RAM_ID,
    SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_MAIN_SYSCLK0_4_CLK_EDC_CTRL_CBASS_INT_MAIN_SYSCLK0_4_BUSECC_RAM_ID,
};


static SDL_ECC_InitConfig_t ECC_Test_AGGR1A0ECCInitConfig =
{
    .numRams = MAIN_AGGR1_AGGR1_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_AGGR1_A0subMemTypeList[0]),
    /**< Sub type list  */
};


static uint32_t arg;
/*********************************************************************
* @fn      ECC_Example_init
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/
int32_t ECC_Example_init (void)
{
    int32_t retValue=0;
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;

    #if defined (R5F_CORE)
    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

    if (retValue == 0) {
            /* Initialize ECC Memory */
        result = SDL_ECC_initMemory(SDL_ECC_AGGR1, SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_IK3VTM_N16FFC_MAIN_0_VBUSP_P2P_BRIDGE_IK3VTM_N16FFC_MAIN_0_VBUSP_BRIDGE_BUSECC_RAM_ID);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing Memory of R5FSS0 CORE0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete \r\n");
        }
    }
    #endif


    if (retValue == SDL_APP_TEST_PASS) {
        /* Initialize MAIN ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_APP_TEST_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Example_init: Error initializing MAIN ESM: result = %d\n\n", result);

            retValue = SDL_APP_TEST_FAILED;
        } else {
            DebugP_log("ECC_Example_init: Init MAIN ESM complete \n\n");
        }
    }

    if (retValue == SDL_APP_TEST_PASS) {
        /* Initialize MCU ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &ECC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_APP_TEST_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Example_init: Error initializing MCU ESM: result = %d\n\n", result);

            retValue = SDL_APP_TEST_FAILED;
        } else {
            DebugP_log("ECC_Example_init: Init MCU ESM complete \n\n");
        }
    }

    if (retValue == SDL_APP_TEST_PASS) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_ECC_AGGR1, &ECC_Test_AGGR1A0ECCInitConfig);
        if (result != SDL_APP_TEST_PASS) {
            /* print error and quit */
            DebugP_log("SDTF_init: Error initializing M4F core ECC: result = %d\n\n", result);

            retValue = SDL_APP_TEST_FAILED;
        } else {
            DebugP_log("\n\nSDTF_init: AGGR1 ECC Init complete \n\n");
        }
    }

    return retValue;
}


/*********************************************************************
 * @fn    runECC2BitAGGR1InjectTest
 *
 * @brief   Execute ECC Doule bit error example test on AGGR1 ECC aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t runECC2BitAGGR1_RAMInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=SDL_APP_TEST_PASS;
    uint32_t subType;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));

    DebugP_log("\n\n AGGR1 Double bit error inject Example test UC-2: starting");

    /* Run one shot test for AGGR1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x100u);

    injectErrorConfig.flipBitMask = 0x5;
    injectErrorConfig.chkGrp = 0x1;

    subType = SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_IK3VTM_N16FFC_MAIN_0_VBUSP_P2P_BRIDGE_IK3VTM_N16FFC_MAIN_0_VBUSP_BRIDGE_BUSECC_RAM_ID;

    result = SDL_ECC_injectError(SDL_ECC_AGGR1,
                                subType,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_APP_TEST_PASS ) {
        DebugP_log("\n\n AGGR1  Double bit error inject test: Subtype %d: test failed",
                    subType);
        retVal = SDL_APP_TEST_FAILED;
    } else {
        DebugP_log("\n\n AGGR1 Double bit error inject test: Subtype 0x%p test complete",
                    subType);
    }

    return retVal;
}

int32_t runECC2BitAGGR1_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t subType;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));

    DebugP_log("\n\n AGGR1 Double bit error inject Example test UC-1: starting");

    /* Run one shot test for AGGR1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0u);

    injectErrorConfig.flipBitMask = 0x5;
    injectErrorConfig.chkGrp = 0x1;

    subType = SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_MAIN_SYSCLK0_4_CLK_EDC_CTRL_CBASS_INT_MAIN_SYSCLK0_4_BUSECC_RAM_ID;

    result = SDL_ECC_injectError(SDL_ECC_AGGR1,
                              subType,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig);

    if (result != SDL_APP_TEST_PASS ) {
        DebugP_log("\n\n AGGR1  Double bit error inject test: Subtype %d: test failed",
                    subType);
        retVal = SDL_APP_TEST_FAILED;
    } else {
        DebugP_log("\n\n AGGR1 Double bit error inject test: Subtype 0x%p test complete",
                    subType);
    }

    return retVal;
}

int32_t ECC_Test_runECC1BitAGGR1ParityInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t subType;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

     DebugP_log("\n\n\n\n**** AGGR1 Memory Parity TEST ****\n\n");

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));

    /* Run one shot test for AGGR1_BUSECC_RAM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0u);

    injectErrorConfig.flipBitMask = 0x1;
    injectErrorConfig.chkGrp = SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_MAIN_SYSCLK0_4_CLK_EDC_CTRL_CBASS_INT_MAIN_SYSCLK0_4_BUSECC_GROUP_3_ID;
    subType = SDL_ECC_AGGR1_IAM64_MAIN_INFRA_CBASS_CBASS_MAIN_0_AM64_MAIN_INFRA_CBASS_CBASS_MAIN_SYSCLK0_4_CLK_EDC_CTRL_CBASS_INT_MAIN_SYSCLK0_4_BUSECC_RAM_ID;

    result = SDL_ECC_injectError(SDL_ECC_AGGR1,
                                subType,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_APP_TEST_PASS ) {
        DebugP_log("\n\n AGGR1  Single bit error self test: Subtype %d: test failed",
                    subType);
        retVal = SDL_APP_TEST_FAILED;
    } else {
        DebugP_log("\n\n AGGR1 Single bit error self test: Subtype 0x%p test complete",
                    subType);
    }

    return retVal;
}


static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 10000;
    uint32_t timeOutCnt = 0;

    DebugP_log("\n\n ESM Safety Example tests: starting");

    if (retVal == SDL_APP_TEST_PASS)
    {
        result = runECC2BitAGGR1_InjectTest();
        if (result == SDL_APP_TEST_PASS)
        {
        DebugP_log("\n\n Waiting for ESM Interrupt \n\n");
        do
            {
                /* dummy wait for the interrupt */
                SDL_DPL_delay(DELAY);;
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if(result == SDL_APP_TEST_PASS){
            DebugP_log("\n\nUC-1: Got Low priority ESM Interrupt \n\n");
            esmError = false;
        }

        if (result != SDL_APP_TEST_PASS) {
            retVal = SDL_APP_TEST_FAILED;
            DebugP_log("\n\nESM_ECC_Example_run: UC-1 has failed...");
            /* UC-1 Low priority AGGR1 interrupt */
        }
    }

    if (retVal == SDL_APP_TEST_PASS)
    {
        result = runECC2BitAGGR1_RAMInjectTest();
        if (result == SDL_APP_TEST_PASS)
        {
            DebugP_log("\n\n Waiting for ESM Interrupt \n\n");
        do
            {
                /* dummy wait for the interrupt */
                SDL_DPL_delay(DELAY);;
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if(result == SDL_APP_TEST_PASS){
            DebugP_log("\n\n UC-2: Got High priority ESM Interrupt \n\n");
            esmError = false;
        }

        if (result != SDL_APP_TEST_PASS) {
            retVal = SDL_APP_TEST_FAILED;
            DebugP_log("\n\n ESM_ECC_Example_run: UC-2 has failed....");
            /* UC-2 High priority AGGR1 interrupt */
        }
    }

    if (retVal == SDL_APP_TEST_PASS)
    {
        result = ECC_Test_runECC1BitAGGR1ParityInjectTest();
        if (result == SDL_APP_TEST_PASS)
        {
        DebugP_log("\n\n Waiting for ESM Interrupt \n\n");
        do
            {
                /* dummy wait for the interrupt */
                SDL_DPL_delay(DELAY);;
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmError == false);
        }
        if(result == SDL_APP_TEST_PASS){
            DebugP_log("\n\n Memory Parity Error Test Complete \n\n");
            esmError = false;
        }

        if (result != SDL_APP_TEST_PASS) {
            retVal = SDL_APP_TEST_FAILED;
            DebugP_log("\n\n Memory Parity Error Test has failed...");
        }
    }


    if ( retVal == SDL_APP_TEST_PASS) {
        DebugP_log("\n\n ECC Safety Example tests: success");
    } else {
        DebugP_log("\n\n ECC Safety Example tests: failed");
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
        DebugP_log("\n\n ECC Safety Example tests: unsuccessful");
        return SDL_EFAIL;
    }

    testResult = ECC_sdlFuncTest();


    return (testResult);
}

/* Nothing past this point */
