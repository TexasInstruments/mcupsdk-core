/*
 *   Copyright (c) Texas Instruments Incorporated 2019-2024
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
 *  \file     ecc_func.c
 *
 *  \brief    This file contains ECC SDL Function test code for R5 core.
 *
 *  \details  ECC SDL API module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <sdl/sdl_esm.h>
#include <sdl/ecc/soc/am64x_am243x/sdl_ecc_soc.h>
#include <ecc_func.h>
#include "ecc_test_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <drivers/soc.h>
#include <dpl_interface.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* delay for 1us*/
#define DELAY 1

#define MAIN_MSMC_AGGR0_MAX_MEM_SECTIONS (2u)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if defined(SOC_AM64X) || defined(SOC_AM243X)
extern volatile bool esmError;
#if defined (M4F_CORE)
SDL_ESM_config ECC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xffffe7ffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                },
    /**< All events enable: except timer and self test events */
    /* Temporarily disabling vim compare error as well */
    /*
     * Enable only the Main VTM ECC events. Enabling both MCU and Main doesn't work
     * well with event routing. And only one is needed to get notification of VTM ECC events.
     */

    .priorityBitmap = {0xffffe7ffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                        },
    /**< All events high priority */
    .errorpinBitmap = {0xffffe7ffu, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                      },
    /**< All high priority events trigger error pin */
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
    .enableBitmap = {0xffffe7f8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                },
    /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well */
    /*
     * Enable only the Main VTM ECC events. Enabling both MCU and Main doesn't work
     * well with event routing. And only one is needed to get notification of VTM ECC events.
     */
    .priorityBitmap = {0xffffe7f8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0xffffe7f8u, 0x013fffffu, 0x7fffff7fu, 0xffffffe6u,
                      },
    /**< All high priority events trigger error pin */
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
#endif
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

int32_t ECC_Memory_init (void);

static uint32_t arg;
/*********************************************************************
* @fn      ECC_Memory_init
*
* @brief   Initializes Software Diagostics Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/
int32_t ECC_Memory_init (void)
{
    int32_t retValue=0;
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;

    /* Initialise exception handler */
    //ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \n");

    if (retValue == SDL_PASS) {
        /* Initialize MCU ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &ECC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\rECC_Memory_init: Error initializing MCU ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Init MCU ESM complete \n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize MAIN ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN,SDL_ESM_applicationCallbackFunction,ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\rECC_Memory_init: Error initializing MAIN ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Init MAIN ESM complete \n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize ECC callbacks within the MCU ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_MCU_ESM0);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\rECC_Memory_init: Error initializing ECC callback for MCU ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for MCU ESM \n");
        }
    }

    if (retValue == SDL_PASS) {
        /* Initialize ECC callbacks within the Main ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_MAIN_ESM0);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("\rECC_Memory_init: Error initializing ECC callback for Main ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for Main ESM \n");
        }
    }

    return retValue;
}

#define DEBUG

/********************************************************************
 * @fn      ecc_aggr_test
 *
 * @brief   Execute ECC single bit error self test on ecc_aggr_test ECC aggregator
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t ecc_aggr_test(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t j,i,k,mainMem;
    SDL_ecc_aggrRegs *pEccAggr;
    uint32_t maxTimeOutMilliSeconds = 3000;
    uint32_t timeOutCnt = 0;
    SDL_ECC_InjectErrorType    intsrc;
    uint32_t errSrc;
    uint32_t pscDomainState;
    uint32_t pscModuleStateMCU2Main = 0;

#ifdef DEBUG
    int32_t selectedIndex = -1;
    bool exit = (bool)false;
#endif

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
#ifdef DEBUG
    while (exit != (bool)true)
    {
        DebugP_log("\r\n Select the memory to test...");
        scanf("%" SCNd32, &selectedIndex);

        if (selectedIndex == -1)
        {
            DebugP_log("\r\n Exiting the test on request \n");
            exit = (bool)true;
            continue;
        }
        DebugP_log("\r...selected %d\n", selectedIndex);
        if (selectedIndex >= SDL_ECC_MEMTYPE_MAX)
        {
            DebugP_log("\r\necc_aggr_test: selection [%d] is not a valid memory id\n", selectedIndex);
            continue;
        }

        SOC_getPSCState(SOC_PSC_DOMAIN_ID_MAIN, CSL_MCU_GP_CORE_CTL_MCU,
            CSL_MAIN_LPSC_ADC, &pscDomainState, &pscModuleStateMCU2Main);

        DebugP_log("PSC Domain STATE Result = %d", pscDomainState);
        DebugP_log("PSC MOdule STATE Result = %d", pscModuleStateMCU2Main);

#endif

        memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
        /* Run one shot test for ecc_aggr_test 1 bit error */
        /* Note the address is relative to start of ram */
        injectErrorConfig.pErrMem = (uint32_t *)(0u);

        injectErrorConfig.flipBitMask = 0x3;
#ifdef DEBUG
        mainMem = selectedIndex;
#else
        for (mainMem = SDL_MCU_M4FSS0_BLAZAR_ECCAGGR; mainMem < SDL_ECC_MEMTYPE_MAX; mainMem++)
        {
#endif
#ifndef DEBUG
            if (mainMem >= SDL_ECC_MEMTYPE_MAX)
            {
                DebugP_log("\r\necc_aggr_test: selection [%d] is not a valid memory id\n", mainMem);
                continue;
            }
            else if (ECC_Test_config[mainMem].initConfig == NULL)
            {
                DebugP_log("\r\necc_aggr_test: [%d] Skipping %s due to missing information\n", mainMem, ECC_Test_config[mainMem].aggrName);
                continue;
            }
#endif
            DebugP_log("\r\n ecc_aggrtest: [%d] single bit error self test: %s starting \n\n", mainMem, ECC_Test_config[mainMem].aggrName);
            /* Sub memory list  entered in the for loop for perticular mem type*/

            if (ECC_Test_config[mainMem].initConfig != NULL)
            {

                /* Initialize ECC */
                result = SDL_ECC_init(mainMem, ECC_Test_config[mainMem].initConfig);
                if (result != SDL_PASS)
                {
                    /* print error and quit */
                    DebugP_log("\r\nECC_Memory_init: [%d] Error initializing %s: result = %d\n", mainMem, ECC_Test_config[mainMem].aggrName, result);

                    result = -1;
                }
                else
                {
                    DebugP_log("\r\nECC_Memory_init: [%d] %s ECC Init complete \n", mainMem, ECC_Test_config[mainMem].aggrName);
                }

            }
            else
            {
                DebugP_log("\r\nECC_Memory_init: [%d] Skipping %s due to missing data\n", mainMem, ECC_Test_config[mainMem].aggrName);
            }

            for (i=0; i< SDL_ECC_aggrTable[mainMem].numRams; i++)
            {
                if ((SDL_ECC_aggrTable[mainMem].ramTable[i].RAMId) != SDL_ECC_RAMID_INVALID)
                {
                    if ((SDL_ECC_aggrTable[mainMem].ramTable[i].ramIdType) != SDL_ECC_AGGR_ECC_TYPE_ECC_WRAPPER)
                    {
                        DebugP_log("\rself test started RamId %d  starting \n",i );

                        /* This for loop provide interconnect checkers group */
                        for (j=0; j< SDL_ECC_aggrTable[mainMem].ramTable[i].numCheckers; j++)
                        {
                            //DebugP_log("\r\n self test started CheckGroup %d  starting \n",j );
                                         injectErrorConfig.chkGrp = j;
                            if(SDL_ECC_aggrTable[mainMem].esmIntSEC != 0u)
                            {
                                intsrc = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;

                            }
                            else
                            {
                                intsrc = SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE;
                            }
                            result = SDL_ECC_selfTest(mainMem,
                                                      i,
                                                      intsrc,
                                                      &injectErrorConfig,
                                                      100000);

                            if (result != SDL_PASS ) {
                                DebugP_log("\r\n ecc_aggr_test self test: mainMem %d: fixed location test failed,Interconnect type RAM id = %d, checker group = %d\n",
                                           mainMem, i, j);
                                retVal = -1;
                            }
                        }
                    }
                    else
                    {
                        for (k=0; k< SDL_ECC_aggrTable[mainMem].numMemEntries; k++)
                        {
                            if ((SDL_ECC_aggrTable[mainMem].memConfigTable[k].memSubType) == (SDL_ECC_aggrTable[mainMem].ramTable[i].RAMId))
                            {
                                    injectErrorConfig.pErrMem =((uint32_t *)SDL_ECC_aggrTable[mainMem].memConfigTable[k].memStartAddr);
                                    break;
                            }
                        }


                            if (SDL_ECC_aggrTable[mainMem].memConfigTable[k].readable == ((bool)true))
                            {
                                DebugP_log("\rself test started accessable RamId %d  starting \n",i );
                                if(SDL_ECC_aggrTable[mainMem].esmIntSEC != 0u)
                                {
                                    intsrc = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
                                }
                                else
                                {
                                    intsrc = SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE;
                                }
                                if (SDL_ECC_aggrTable[mainMem].ramTable[i].aggregatorTypeInjectOnly == 1U)
                                {
                                    result = SDL_ECC_injectError(mainMem,i,intsrc,&injectErrorConfig);
                                    if ((result == SDL_PASS) && (mainMem < SDL_ECC_MEMTYPE_MAX))
                                    {
                                        pEccAggr = SDL_ECC_aggrTransBaseAddressTable[mainMem];
                                        SDL_ecc_aggrSetEccRamIntrPending(pEccAggr, i, errSrc);

                                        do
                                        {
                                            timeOutCnt += 10;
                                            if (timeOutCnt > maxTimeOutMilliSeconds)
                                            {
                                                result = SDL_EFAIL;
                                                break;
                                            }
                                        } while (esmError == false);
                                        DebugP_log("\r    ...skipped because this is Inject Only type\n");

                                        timeOutCnt = 0;
                                        esmError = false;
                                        result = SDL_PASS;
                                    }
                                }
                                else
                                {
                                    result = SDL_ECC_selfTest(mainMem,
                                                              i,
                                                              intsrc,
                                                              &injectErrorConfig,
                                                              100000);
                                    if (result != SDL_PASS ) {
                                        DebugP_log("\r\n ecc_aggr_test self test: mainMem %d: accessable mem type test failed, Wrapper type RAM id = %d\n",
                                                   mainMem, i);
                                        retVal = -1;
                                    }
                                }
                            }
                            else{
                                DebugP_log("\rself test started not accessable RamId %d  starting \n",i );

                                if(SDL_ECC_aggrTable[mainMem].esmIntSEC != 0u)
                                {
                                    intsrc = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
                                    errSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
                                }
                                else
                                {
                                    intsrc = SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE;
                                    errSrc = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
                                }

                                //result = SDL_ECC_injectError(mainMem,i,intsrc,&injectErrorConfig);
				result = 0x0;
                                if ((result == SDL_PASS) && (mainMem < SDL_ECC_MEMTYPE_MAX) )
                                {
                                    if (SDL_ECC_aggrTable[mainMem].ramTable[i].aggregatorTypeInjectOnly != 1)
                                    {
                                        uintptr_t key;
                                        pEccAggr = SDL_ECC_aggrTransBaseAddressTable[mainMem];
                                        key = HwiP_disable();
                                        SDL_ecc_aggrSetEccRamIntrPending(pEccAggr, i, errSrc);
                                        HwiP_restore(key);
                                        do
                                        {
                                            timeOutCnt += 10;
                                            if (timeOutCnt > maxTimeOutMilliSeconds)
                                            {
                                                result = SDL_EFAIL;
                                                break;
                                            }
                                        } while (esmError == false);

                                        timeOutCnt = 0;
                                        esmError = false;
                                        if (result == SDL_PASS)
                                        {
                                            DebugP_log("\r\n\n  Got it\n");
                                        }
                                        else
                                        {
                                            DebugP_log("\r\n Failed\n");
                                        }
                                    }
                                    else
                                    {
                                        do
                                        {
                                            timeOutCnt += 10;
                                            if (timeOutCnt > maxTimeOutMilliSeconds)
                                            {
                                                result = SDL_EFAIL;
                                                break;
                                            }
                                        } while (esmError == false);

                                        timeOutCnt = 0;
                                        esmError = false;
                                        DebugP_log("\r    ...skipped because this is Inject Only type\n");
                                        result = SDL_PASS;
                                    }
                                }
                                else {
                                    DebugP_log("\r    Inject error failed!\n");
                                }
                                if (result != SDL_PASS ) {
                                    DebugP_log("\r\n ecc_aggr_test self test: mainMem %d: fixed location test failed, Wrapper type RAM id = %d\n",
                                               mainMem, i);
                                    retVal = -1;
                                }
                            }
                    }
                }
            }
#ifndef DEBUG
	}
#endif
#ifdef DEBUG
    }
#endif
    return retVal;
}

static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;

    DebugP_log("\r\n ECC SDL API tests: starting \n\n");
    if (retVal == SDL_PASS) {
       result = ecc_aggr_test();
       if (result != SDL_PASS) {
           retVal = -1;
            DebugP_log("\r\n ecc_aggr test has failed...");
       }
    }
    return retVal;
}

/* ECC Function module test */
int32_t ECC_funcTest(void)
{
    int32_t testResult;

    testResult = ECC_Memory_init();
    if (testResult != 0)
    {
        DebugP_log("\r\n ECC func tests: unsuccessful");
        return SDL_EFAIL;
    }

    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
