/*
 *   Copyright (c) Texas Instruments Incorporated 2019-2023
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
 *  \file     ecc_test_func.c
 *
 *  \brief    This file contains ECC SDL Function test code for R5 core.
 *
 *  \details  ECC SDL API module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"
#include <sdl/sdl_esm.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
#include <dpl_interface.h>
#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/ecc/soc/am64x_am243x/sdl_ecc_soc.h>


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if defined (M4F_CORE)
SDL_ESM_config ECC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xffffffffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0xffffffffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0xffffffffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xfffffb0fu, 0xf7c0000fu, 0xffbffd8f, 0x00008f80u,
                     0x00001800u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0xfffffb0fu, 0xf7c0000fu, 0xffbffd8f, 0x00008f80u,
                       0x00001800u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xfffffb0fu, 0xf7c0000fu, 0xffbffd8f, 0x00008f80u,
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
SDL_ESM_config ECC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0xfffffff8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xfffffb0fu, 0xf7c000efu, 0xffbffd8f, 0x00008f80u,
                     0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0xfffffb0fu, 0xf7c0006fu, 0xffbffd8f, 0x00008f80u,
                       0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xfffffb0fu, 0xf7c0006fu, 0xffbffd8f, 0x00008f80u,
                       0x00001b80u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

#define SDL_PMU_CTR_MAX_VALUE (0xffffffffu)

#define SDL_ECC_ATCM_SINGLE_BIT_ERROR_EVENT (0x67u)
#endif

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
volatile uint32_t testLocationValue;

#if defined (M4F_CORE)
SDL_ECC_MemType geccMemType = SDL_MCU_M4FSS0_BLAZAR_ECCAGGR;
#endif

#if defined (R5F_CORE)
SDL_ECC_MemType geccMemType = SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR;
#endif

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Defines */
#define MAX_MEM_SECTIONS   (1u)

#define MCAN1_MCANSS_MSGMEM_ADD   (0x020718000u)


/* Function prototypes */
void ECC_Test_copyResetVector(void);
int32_t ECC_Test_EsmInitHandlerInit(SDL_ESM_Inst esmInstType);
int32_t ECC_Test_init (void);

#if defined (M4F_CORE)
static SDL_ECC_MemSubType ECC_Test_CoresubMemTypeList[MAX_MEM_SECTIONS] =
{
  SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
};
#endif

#if defined (R5F_CORE)
static SDL_ECC_MemSubType ECC_Test_CoresubMemTypeList[MAX_MEM_SECTIONS] =
{
  SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
};

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

static SDL_ECC_InitConfig_t ECC_Test_CoreECCInitConfig =
{
    .numRams = MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_CoresubMemTypeList[0]),
    /**< Sub type list  */
};

#define AGGR1_MEM_SECTIONS (2u)
static SDL_ECC_MemSubType ECC_Test_AGGR1_A0subMemTypeList[AGGR1_MEM_SECTIONS] =
{
    SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
    SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MAINMSMCA0ECCInitConfig =
{
    .numRams = AGGR1_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_AGGR1_A0subMemTypeList[0]),
    /**< Sub type list  */
};

#endif

static uint32_t arg;
/*********************************************************************
* @fn      ECC_Test_init
*
* @brief   Initializes Software Diagostics Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/
int32_t ECC_Test_init (void)
{
    int32_t retValue=0;
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;
    SDL_ECC_staticRegs staticRegs;
#if defined (R5F_CORE)
    /*Enabling the ECC module*/
     SDL_ECC_UTILS_enableECCATCM();

     /*Enabling the B0TCM ECC module*/
   	SDL_ECC_UTILS_enableECCB0TCM();

     /*Enabling the Event bus*/
     SDL_UTILS_enable_event_bus();

     /* Initialise exception handler */
     ECC_Test_exceptionInit();

     /* enable PMU event monitoring of ATCM 1 bit*/
                       SDL_ECC_UTILS_configSecIntr(SDL_PMU_CTR_MAX_VALUE,
                                                 SDL_ECC_ATCM_SINGLE_BIT_ERROR_EVENT,
                                                 1);
#endif

    DebugP_log("\r\nECC_Test_init: UART ready to print, proceeding with ECC_Test init \n\n");

    DebugP_log("\r\nECC_Test_init: Profile init complete \n\n");
#if defined (R5F_CORE)
    if (retValue == SDL_PASS) {
             /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR, SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing Memory of R5FSS0 CORE0 ECC: result = %d\n", result);

             retValue = -1;
         } else {
             DebugP_log("\nECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete \n");
         }
       }
#endif
    if (retValue == SDL_PASS) {
        /* Initialize MCU ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &ECC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing MCU ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Init MCU ESM complete \n\n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize MAIN ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN,SDL_ESM_applicationCallbackFunction,ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MAIN ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Init MAIN ESM complete \n\n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize ECC */
        result = SDL_ECC_init(geccMemType, &ECC_Test_CoreECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing M4F core ECC: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: M4F Core ECC Init complete \n\n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Read back the static registers */
        result = SDL_ECC_getStaticRegisters(geccMemType, &staticRegs);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error reading the static registers: result = %d\n\n");

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: M4F Memtype Register Readback successful \n\n");
        }
    }

    /* Initialize an ECC aggregator type that requires mapping.
     * This example only shows MSMC_AGGR0 instance.*/
    if (retValue == SDL_PASS) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MAINMSMCA0ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing MCAN1 ECC: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN1 ECC Init complete \n\n");
        }
    }

    /* Test the path for MSMC mapping */
    if (retValue == SDL_PASS) {
        /* Read back the static registers */
        result = SDL_ECC_getStaticRegisters(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR, &staticRegs);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error reading the static registers: result = %d\n\n");

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN1 Memtype Register Readback successful \n\n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize ECC callbacks within the MCU ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_MCU_ESM0);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing ECC callback for MCU ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for MCU ESM \n\n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize ECC callbacks within the Main ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_MAIN_ESM0);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing ECC callback for Main ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for Main ESM \n\n");
        }
    }


    if (retValue == SDL_PASS) {
        result = SDL_ECC_initMemory(geccMemType, 1U);
        if (result != SDL_PASS) {
             /* print error and quit */
              DebugP_log("\r\nECC_Test_init: Error initializing ECC memory: retValue = %d\n", retValue);
             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: ECC Memory init complete \n\n");
         }
    }

    return retValue;
}

/*********************************************************************
 * @fn      ECC_Test_runECC1BitInjectTest
 *
 * @brief   Execute ECC 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Single bit error inject: test starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00);

    /* Run one shot test for M4FSS0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    #if defined (M4F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
    #endif
    #if defined (R5F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
    #endif


    if (result != SDL_PASS ) {
        DebugP_log("\r\n Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\n Single bit error inject at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

/*********************************************************************
 * @fn      ECC_Test_runECC2BitInjectTest
 *
 * @brief   Execute ECC 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC2BitInjectTest(void)
{
    SDL_ErrType_t result;
    volatile  int32_t retVal=0;
    volatile int32_t delayInject;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;


    DebugP_log("\r\n Double bit error inject: starting\n");

    /* Run one shot test for M4FSS0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x200u);

    injectErrorConfig.flipBitMask = 0x101;
    for(delayInject=0;delayInject<0xfffu;delayInject++);
    #if defined (M4F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
    #endif

    #if defined (R5F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
    #endif
    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
        DebugP_log("\r\n Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\r\n Double bit error inject: pErrMem 0x%p fixed location once test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}



/*********************************************************************
 * @fn      ECC_Test_runECC1BitSelfTest
 *
 * @brief   Execute ECC 1 bit Self test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC1BitSelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Single bit error self test: starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x100);

    /* Run one shot test for M4FSS0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;

    #if defined (M4F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
                              100000);
    #endif
    #if defined (R5F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
                              100000);
    #endif
    if (result != SDL_PASS ) {
         DebugP_log("\r\n Single bit error self test at pErrMem 0x%p test failed",
                     injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\r\n Single bit error self test at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

int32_t ECC_Test_runECC1BitNeg1SelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Single bit error negative-1 self test: starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x100);

    /* To get EFAIL  passing invalid chkgrp*/
    injectErrorConfig.chkGrp = 50U;
    #if defined (M4F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              28U,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
                              100000);
    #endif

    #if defined (R5F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              30U,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
                              100000);
    #endif

    if (result != SDL_EFAIL ) {
         DebugP_log("\r\n Single bit error negative-1 self test at pErrMem 0x%p test failed",
                     injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\r\n Single bit error negative-1 self test at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

/*********************************************************************
 * @fn      ECC_Test_runECC2BitSelfTest
 *
 * @brief   Execute ECC 2 bit Self Test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC2BitSelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Double bit error self test: starting\n");

    /* Run one shot test for M4FSS0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x100);

    injectErrorConfig.flipBitMask = 0x101;
    #if defined (M4F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig,
                              1000);
    #endif

    #if defined (R5F_CORE)
    result = SDL_ECC_selfTest(geccMemType,
                              SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig,
                              1000);
    #endif

    if (result != SDL_PASS ) {
        DebugP_log("\r\n Double bit error self test: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        DebugP_log("\r\n Double bit error self test: pErrMem 0x%p fixed location once test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

/*********************************************************************
 * @fn      ECC_Test_runECC1BitMCAN1SelfTest
 *
 * @brief   Execute ECC Single bit error self test on MSMC_BUSECC_RAM aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC1BitMCAN1SelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t subType;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Single bit error self test: starting\n");

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));

    /* Run one shot test for SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(MCAN1_MCANSS_MSGMEM_ADD);

    injectErrorConfig.flipBitMask = 0x1;
    injectErrorConfig.chkGrp = 0x4;
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    subType = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID;
#endif
    result = SDL_ECC_selfTest(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                              subType,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
                              1000);


    if (result != SDL_PASS ) {
        DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR  Single bit error self test: Subtype %d: test failed",
                    subType);
        retVal = -1;
    } else {
        DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Single bit error self test: Subtype 0x%p test complete",
                    subType);
    }

    return retVal;
}

/*********************************************************************
 * @fn      ECC_Test_runECC2BitMCAN1SelfTest
 *
 * @brief   Execute ECC Double bit error self test on MSMC_BUSECC_RAM ECC aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECC2BitMCAN1SelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t subType, mainMem;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Double bit error self test: starting\n");

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
	/* Run one shot test for MSMC_BUSECC_RAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(MCAN1_MCANSS_MSGMEM_ADD);


#if defined(SOC_AM64X) || defined(SOC_AM243X)
    injectErrorConfig.flipBitMask = 0x3;
    injectErrorConfig.chkGrp = 0x1;
	mainMem = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR;
    subType = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID;
#endif

    result = SDL_ECC_selfTest(mainMem,
                              subType,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig,
							  10000);

	#if defined(SOC_AM64X) || defined(SOC_AM243X)
    if (result != SDL_PASS ) {
        DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR  Double bit error self test: Subtype %d: fixed location once test failed",
                    subType);
        retVal = -1;
    } else {
        DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Double bit error self test: Subtype 0x%p fixed location once test complete",
                    subType);
    }
	#endif

    return retVal;
}

#if defined(SOC_AM64X) || defined(SOC_AM243X)
static bool ECC_Test_ECC_DEDTriggerFlag = false;
/*********************************************************************
 *
 * @brief   indicate DED trigger
 *
 *
 * @return  0 : Success; < 0 for failures
 */
void ECC_Test_ECC_indicateDEDTrigger(void)
{
    ECC_Test_ECC_DEDTriggerFlag = true;
}

int32_t ECC_Test_runECC_ErrMemInjectTest(void) //DOnt add for R5f because btcm is not accessable
{
    #if defined (M4F_CORE)
    SDL_ErrType_t result;
    #endif
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Error inject: EFAIL test starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00);

    /* Run one shot test for M4FSS0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    #if defined (M4F_CORE)
      result = SDL_ECC_injectError(geccMemType,
                                   SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IDRAM_ECC_RAM_ID,
                                   SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                   &injectErrorConfig);



    if (result != SDL_EFAIL ) {
        DebugP_log("\r\n Error inject at pErrMem 0x%p EFAIL test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\n Error inject at pErrMem 0x%p: EFAIL test complete",
                    injectErrorConfig.pErrMem);
    }
    #endif
    #if defined (R5F_CORE)
      DebugP_log("\r\n This test is not executed because BTCM is not accessable");
      retVal = SDL_PASS;
    #endif
    return retVal;
}

int32_t ECC_Test_runECC1BitNrowInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Single bit N row error inject: test starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00);

    /* Run one shot test for M4FSS0 1 bit N row error */
    injectErrorConfig.flipBitMask = 0x10;
    #if defined (M4F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                                 &injectErrorConfig);
    #endif
    #if defined (R5F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                                 &injectErrorConfig);
    #endif

    if (result != SDL_PASS ) {
        DebugP_log("\r\n Single bit N row error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\n Single bit N row error inject at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

int32_t ECC_Test_runECC1BitRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Single bit Repeat error inject: test starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00);

    /* Run one shot test for M4FSS0 1 bit Repeat error */
    injectErrorConfig.flipBitMask = 0x10;
    #if defined (M4F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                                 &injectErrorConfig);
    #endif

    #if defined (R5F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                                 &injectErrorConfig);
    #endif

    if (result != SDL_PASS ) {
        DebugP_log("\r\n Single bit Repeat error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\n Single bit Repeat error inject at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

int32_t ECC_Test_runECC2BitNrowInjectTest(void)
{
    volatile SDL_ErrType_t result;
    volatile  int32_t retVal=0;
    volatile uint32_t delay;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n Double bit N row error inject: test starting\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00);

    /* Run one shot test for M4FSS0 2 bit N row error */
    injectErrorConfig.flipBitMask = 0x101;
    for(delay=0; delay<0xfffu; delay++);
    #if defined (M4F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_MCU_M4FSS0_BLAZAR_ECC_BLAZAR_IIRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
                                 &injectErrorConfig);
    #endif

    #if defined (R5F_CORE)
    result = SDL_ECC_injectError(geccMemType,
                                 SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
                                 &injectErrorConfig);
    #endif

    if (result != SDL_PASS ) {
        DebugP_log("\r\n Double bit N row error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\n Double bit N row error inject at pErrMem 0x%p: test complete",
                    injectErrorConfig.pErrMem);
    }

    return retVal;
}

#endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
int32_t ECC_Test_runECC2BitRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
   uint32_t subType, mainMem;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Double bit repeat error inject: starting\n");

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
	/* Run one shot test for MSMC_BUSECC_RAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(MCAN1_MCANSS_MSGMEM_ADD);

	injectErrorConfig.flipBitMask = 0x5;
    subType = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID;
	mainMem = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR;

    result = SDL_ECC_injectError(mainMem,
                                subType,
                                SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR  Double bit repeat error inject: Subtype %d: fixed location once test failed",
                    subType);
        retVal = -1;
    } else {
        DebugP_log("\r\n MSMC_BUSECC_RAM Double bit repeat error inject: Subtype 0x%p fixed location once test complete",
                    subType);
    }

    return retVal;
}
#endif

 /*********************************************************************
 * @fn      ECC_Test_runECCSEC_DED_MCAN1SelfTest
 *
 * @brief   Execute ECC Single bit error self test on MSMC_BUSECC_RAM aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ECC_Test_runECCSEC_DED_MCAN1SelfTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t subType, i;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    DebugP_log("\r\n SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR Single bit error self test: starting\n");

    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));

    /* Run one shot test for MSMC_BUSECC_RAM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(MCAN1_MCANSS_MSGMEM_ADD);

    injectErrorConfig.flipBitMask = 0x5;
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    subType = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID;
#endif
	for(i = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE; i<= SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT; i++)
	{
		result = SDL_ECC_selfTest(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                                  subType,
                                  i,
                                  &injectErrorConfig,
                                  1000);

	}
    if (result == SDL_PASS ) {
        DebugP_log("\r\n ECC_Test_runECCSEC_DED_MCAN1SelfTest  Single bit error self test: Subtype %d: test complete",
                    subType);
		retVal = 0;
    } else {
        DebugP_log("\r\n ECC_Test_runECCSEC_DED_MCAN1SelfTest Single bit error self test: Subtype 0x%p test failed",
                    subType);
		retVal = -1;
    }

    return retVal;
}


static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
    injectErrorConfig.pErrMem = (uint32_t *)(0u);
    injectErrorConfig.flipBitMask = 0x3;

    DebugP_log("\r\n ECC SDL API tests: starting\n");

    result = ECC_Test_runECC1BitInjectTest();
    if (result != SDL_PASS) {
        retVal = -1;
        DebugP_log("\r\n ECC_Test_runECC1BitInjectTest has failed...\n");
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC2BitInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC2BitSelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC2BitSelfTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC1BitSelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC1BitSelfTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC1BitNeg1SelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC1BitNeg1SelfTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC1BitMCAN1SelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC1BitMCAN1SelfTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC2BitMCAN1SelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC2BitMCAN1SelfTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC_ErrMemInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC_ErrMemInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC1BitNrowInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC1BitNrowInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC1BitRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC1BitRepeatInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC2BitNrowInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC2BitNrowInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECC2BitRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECC2BitRepeatInjectTest has failed...\n");
        }
    }

    if (retVal == SDL_PASS) {
        result = ECC_Test_runECCSEC_DED_MCAN1SelfTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\n ECC_Test_runECCSEC_DED_MCAN1SelfTest has failed...\n");
        }
    }
    /* This needs to be last as it is destructive */

    if ( retVal == SDL_PASS) {
        DebugP_log("\r\n ECC SDL API tests: success\n");
    } else {
        DebugP_log("\r\n ECC SDL API tests: failed\n");
    }

    return retVal;

}

/* ECC Function module test */
int32_t ECC_funcTest(void)
{
    int32_t testResult;

    testResult = ECC_Test_init();

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\n ECC SDL API tests: unsuccessful\n");
        return SDL_EFAIL;
    }

    testResult = ECC_sdlFuncTest();


    return (testResult);
}

/* Nothing past this point */
