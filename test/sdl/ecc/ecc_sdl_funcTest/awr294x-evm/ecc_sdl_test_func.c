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
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
#include <kernel/dpl/DebugP.h>

#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
#include <sdl/include/awr294x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/awr294x/sdlr_intr_esm_dss.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay for 1us*/
#define DELAY 1

/* This macro shows how many ESM events are configured*/
#define SDL_MCANA_MAX_MEM_SECTIONS                  (1u)
#define SDL_ESM_MAX_MCANA_EXAMPLE_AGGR              (2u)
#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_GROUP_NUM_2                        (2U)
#define SDL_INTR_GROUP_NUM_3                        (3U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

/* Defines */
#define SDL_ESM_MAX_MSS_PARAM_MAP_WORDS             (10)
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (1u)
#define SDL_MSS_MAX_MEM_SECTIONS                    (1u)
#define SDL_MCANA_MAX_MEM_SECTIONS                  (1u)

#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR             (0x02F7c020u)
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

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID

};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCANA_subMemTypeList[SDL_MCANA_MAX_MEM_SECTIONS] =
{
     SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCANA_ECCInitConfig =
{
    .numRams = SDL_MCANA_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCANA_subMemTypeList[0]),
    /**< Sub type list  */
};


extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

/* Event BitMap for ECC ESM callback for MSS */
SDL_ESM_NotifyParams ECC_TestparamsMSS[SDL_ESM_MAX_MSS_PARAM_MAP_WORDS] =
{
     {
          /* Event BitMap for ECC ESM callback for R5FA Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ATCM0_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for R5FA Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_3,
          .errorNumber = SDL_ESMG3_ATCM0_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for R5FB Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGB_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for R5FB Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGB_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MSS Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MSS Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_ECCAGGMSS_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MCANA Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANA_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MCANA Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANA_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MCANB Single bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANB_SERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },
     {
          /* Event BitMap for ECC ESM callback for MCANB Double bit*/
          .groupNumber = SDL_INTR_GROUP_NUM_1,
          .errorNumber = SDL_ESMG1_MCANB_UERR,
          .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
          .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
          .callBackFunction = &SDL_ESM_applicationCallbackFunction,
     },

};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ECC_Example_init function */
int32_t ECC_Test_MCANA_init (void);
int32_t ECC_Test_R5F_init (void);

void ECC_Test_undefInstructionExptnCallback(void)
{
    printf("\n\n Undefined Instruction exception\n\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    printf("\n\n Software interrupt exception\n\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    printf("\n\n Prefetch Abort exception\n\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    printf("\n\n Data Abort exception\n\n");
}
void ECC_Test_irqExptnCallback(void)
{
    printf("\n\n Irq exception\n\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    printf("\n\n Fiq exception\n\n");
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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      ECC_Test_MCANA_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Test_MCANA_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;

    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MSS_MCANA_ECC, SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing Memory of MCANA ECC: result = %d\n", result);

             retValue = -1;
         } else {
             DebugP_log("\nECC_Test_init: Initialize of MCANA ECC Memory is complete \n");
         }
    }

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MSS_MCANA_ECC, &ECC_Test_MCANA_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing MCANA ECC: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: MCANA ECC initialization is completed \n");
        }
    }
    return retValue;
}/* End of ECC_Test_MCANA_init() */

/*********************************************************************
* @fn      ECC_Test_R5F_init
*
* @brief   Initializes Software Diagnostics Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*********************************************************************/
int32_t ECC_Test_R5F_init (void)
{
    int32_t retValue=0;
    uint8_t u8ParamCount = 0;
    SDL_ErrType_t result;

    /*Enabling the ECC module*/
    SDL_ECC_UTILS_enableECCATCM();

    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\nECC_Test_init: Exception init complete \n");

    if (retValue == 0) {
             /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_R5FSS0_CORE0_ECC_AGGR, SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing Memory of R5FSS0 CORE0 ECC: result = %d\n", result);

             retValue = -1;
         } else {
             DebugP_log("\nECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete \n");
         }
    }
    if (retValue == 0) {
        /* Initialize ESM module */
        for(u8ParamCount = 0; u8ParamCount < SDL_ESM_MAX_MSS_PARAM_MAP_WORDS; u8ParamCount++)
        {
            /* Initialize ESM module */
            result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMSS[u8ParamCount],NULL,NULL);
        }
        if (result == SDL_PASS)
        {
            DebugP_log("\nESM_Test_init: Init MCANA ESM single bit complete \n");
        }
        else {
            DebugP_log("ECC_Example_init: Error initializing ESM for single bit: result = %d\n\n", result);
            result = SDL_EFAIL;
        }
        /*Writing '000' will ungate the ESM_GRP3_ERROR_7 for dounle bit ATCM*/
        SDL_REG32_WR(SDL_MSS_CTRL_ESM_GATING4, (0x0 << (((SDL_ESMG3_ATCM0_UERR ) % 8)*4)));
        gMsmcMemParityInterrupt = false;
    }
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_R5FSS0_CORE0_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing R5FSS0 CORE0 ECC: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: R5FSS0 CORE0 ECC initialization is completed \n");
        }
    }

    if (retValue == 0) {
        /* Initialize ECC callbacks within the Main ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_MSS_ESM);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing ECC callback for MSS ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: ECC Callback Init complete for MSS ESM \n");
        }
    }

    return retValue;
}/* End of ECC_Test_R5F_init() */

/*********************************************************************
 * @fn      ECC_Test_run_MCANA_1BitInjectTest
 *
 * @brief   Execute ECC MCANA  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_selfTest(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                             &injectErrorConfig,
                             10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCANA_1Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCANA 1 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_1Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA  1 N Row bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Single bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_1Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCANA_1Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCANA  1 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_1Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA  1 bit repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Single bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_1Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCANA_1Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCANA  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_1Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA  1 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Single bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_1Bit_N_ROW_RepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCANA_2BitInjectTest
 *
 * @brief   Execute ECC MCANA  2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Run one shot test for MCANA  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                                 SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\n MCANA  Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCANA_2Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCANA 2 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_2Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA 2 N Row bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Double bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_2Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCANA_2Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCANA 2 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_2Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA 2 bit repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Double bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_2Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCANA_2Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCANA 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_2Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x02040000u);

    /* Run one shot test for MCANA 2 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MSS_MCANA_ECC,
                             SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n MCANA  Double bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_2Bit_N_ROW_RepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig,
                                 10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\n R5FSS0 CORE0 ATCM0 BANK0 Single bit error self test inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\n R5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest() */
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

    retVal = ECC_Test_R5F_init();

    if (retVal != 0)
    {
        DebugP_log("\n ECC SDL ECC_Test_R5F_init: unsuccessful");
        return SDL_EFAIL;
    }
    if (retVal == 0) {

        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest has failed...");
        }
    }
    if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest has failed...");
        }
    }


    retVal = ECC_Test_MCANA_init();

    if (retVal != 0)
    {
        DebugP_log("\n ECC SDL ECC_Test_MCANA_init: unsuccessful");
        return SDL_EFAIL;
    }
    if (retVal == 0)
    {

        result = ECC_Test_run_MCANA_1BitInjectTest();
        if(result == SDL_PASS){
            DebugP_log("\n\nMCANA: Injected 1-bit error and got ESM Interrupt \n\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\nESM_ECC_Example_run: MCANA Test has failed...");
            /*Low priority MCANA interrupt */
        }

        result = ECC_Test_run_MCANA_1Bit_N_ROWInjectTest();
        if(result == SDL_PASS){
            DebugP_log("\n\nMCANA: Injected 1-bit N ROW error and got ESM Interrupt \n\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\nESM_ECC_Example_run: MCANA Test has failed...");
            /*Low priority MCANA interrupt */
        }

        result = ECC_Test_run_MCANA_1Bit_Repeat_InjectTest();
        if(result == SDL_PASS){
            DebugP_log("\n\nMCANA: Injected 1-bit Repeat error and got ESM Interrupt \n\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\nESM_ECC_Example_run: MCANA Test has failed...");
            /*Low priority MCANA interrupt */
        }

        result = ECC_Test_run_MCANA_1Bit_N_ROW_RepeatInjectTest();
        if(result == SDL_PASS){
            DebugP_log("\n\nMCANA: Injected 1-bit N Row Repeat error and got ESM Interrupt \n\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\nESM_ECC_Example_run: MCAN Test has failed...");
            /* Low priority MCANA interrupt */
        }
    }
    if (retVal == 0) {

        result = ECC_Test_run_MCANA_2BitInjectTest();
        if(result == SDL_PASS){
            DebugP_log("\n\n MCANA: Injected 2-bit error and got ESM Interrupt\n\n");
            /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n\n ESM_ECC_Example_run: MCANA Test has failed....");
            /*High priority MCANA interrupt */
        }
        result = ECC_Test_run_MCANA_2Bit_N_ROWInjectTest();
         if(result == SDL_PASS){
             DebugP_log("\n\nMCANA: Injected 2-bit N ROW error and got ESM Interrupt \n\n");
             /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
             gMsmcMemParityInterrupt = false;
         }

         if (result != SDL_PASS) {
             retVal = -1;
             DebugP_log("\n\nESM_ECC_Example_run: MCANA Test has failed...");
             /*High priority MCANA interrupt */
         }

         result = ECC_Test_run_MCANA_2Bit_Repeat_InjectTest();
         if(result == SDL_PASS){
             DebugP_log("\n\nMCANA: Injected 2-bit Repeat error and got ESM Interrupt \n\n");
             /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
             gMsmcMemParityInterrupt = false;
         }

         if (result != SDL_PASS) {
             retVal = -1;
             DebugP_log("\n\nESM_ECC_Example_run: MCANA Test has failed...");
             /*High priority MCANA interrupt */
         }

         result = ECC_Test_run_MCANA_2Bit_N_ROW_RepeatInjectTest();
         if(result == SDL_PASS){
             DebugP_log("\n\nMCANA: Injected 2-bit N Row Repeat error and got ESM Interrupt \n\n");
             /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
             gMsmcMemParityInterrupt = false;
         }

         if (result != SDL_PASS) {
             retVal = -1;
             DebugP_log("\n\nESM_ECC_Example_run: MCAN Test has failed...");
             /* High priority MCANA interrupt */
         }
    }

    if ( retVal == 0) {
        DebugP_log("\n ECC SDL API tests: success");
    } else {
        DebugP_log("\n ECC SDL API tests: failed");
    }

    return retVal;
}

/* ECC Function module test */
int32_t ECC_sdl_funcTest(void)
{
    int32_t testResult = 0;

    /*Execute ECC sdl function test*/
    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
