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
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay for 1us*/
#define DELAY 1

/* This macro shows how many ESM events are configured*/
#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_GROUP_NUM_2                        (2U)
#define SDL_INTR_GROUP_NUM_3                        (3U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

/* Defines */
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (3U)
#define SDL_MCAN0_MAX_MEM_SECTIONS                  (1U)
#define SDL_MSS_MAX_MEM_SECTIONS					(1U)

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
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
	SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
	SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,

};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCAN0_subMemTypeList[SDL_MCAN0_MAX_MEM_SECTIONS] =
{
     SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCAN0_ECCInitConfig =
{
    .numRams = SDL_MCAN0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCAN0_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MSSsubMemTypeList[SDL_MSS_MAX_MEM_SECTIONS] =
{
     SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSSECCInitConfig =
{
    .numRams = SDL_MSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSSsubMemTypeList[0]),
    /**< Sub type list  */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* Event BitMap for ECC ESM callback for MAIN */
static uint32_t arg;

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x0018000cu, 0x00018000u, 0x00000000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x00100008u, 0x00010000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x0018000cu, 0x00018000u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ECC_Example_init function */
int32_t ECC_Test_MCAN0_init (void);
int32_t ECC_Test_R5F_init (void);
int32_t ECC_Test_MSS_init (void);

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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      ECC_Test_MCAN0_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Test_MCAN0_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;

    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR, SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCAN0 ECC: result = %d\r\n", result);

             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: Initialize of MCAN0 ECC Memory is complete \r\n");
         }
    }

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MCAN0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCAN0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN0 ECC initialization is completed \r\n");
        }
    }
    return retValue;
}/* End of ECC_Test_MCAN0_init() */

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
    SDL_ErrType_t result;
	SDL_ECC_staticRegs staticRegs;

    /*Enabling the ATCM0 ECC module*/
    SDL_ECC_UTILS_enableECCATCM();
	
	/*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB0TCM();
	
    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_R5FSS0_CORE0_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing R5FSS0 CORE0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 ECC initialization is completed \r\n");
        }
    }
    if (retValue == 0) {
        /* Read back the static registers */
        result = SDL_ECC_getStaticRegisters(SDL_R5FSS0_CORE0_ECC_AGGR, &staticRegs);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error reading the R5FSS0 CORE0 static registers: result = %d\r\n");

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 Memtype Register Readback successful \r\n");
        }
    }
    return retValue;
}/* End of ECC_Test_R5F_init() */

/*********************************************************************
* @fn      ECC_Test_MSS_init
*
* @brief   Initializes Software Diagnostics MSS Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*********************************************************************/
int32_t ECC_Test_MSS_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_SOC_ECC_AGGR, &ECC_Test_MSSECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MSS ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MSS ECC Init complete \r\n");
        }
    }
    return retValue;
}/* End of ECC_Test_MSS_init() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1BitInjectTest
 *
 * @brief   Execute ECC MCAN0  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit error self test: starting \r\n");
			   
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_selfTest(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                             &injectErrorConfig,
                             10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit error self test inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCAN0 1 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit N ROW error inject: starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 N Row bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCAN0  1 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCAN0  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit N ROW Repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2BitInjectTest
 *
 * @brief   Execute ECC MCAN0  2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit error self test: starting \r\n");
		
    /* Run one shot test for MCAN0  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_selfTest(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                                 SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig,
								 10000);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nMCAN0 Double bit error self test: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCAN0 2 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit N ROW error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 N Row bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCAN0 2 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 bit repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCAN0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit N ROW Repeat error inject: starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest() */

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

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error self test: starting \r\n");
		
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

        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error self test at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
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
	
	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: starting \r\n");

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
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B0TCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 single bit error self test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080010u);

    /* Run one shot test for R5FSS0 CORE0 B0TCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                              SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
					    	  10000);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
		/* Access the memory where injection is expected */
		testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 single bit error self testinject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B1TCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nNegative R5FSS0 CORE0 B1TCM0 BANK0 singl bit error self test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00084010u);

    /* Run one shot test for R5FSS0 CORE0 B1TCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig,
								 10000);

    if (result == SDL_PASS ) {
        retVal = -1;
    } else {
        DebugP_log("\r\nNegative R5FSS0 CORE0 B1TCM0 BANK0 singl bit error self test done \r\n");
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_MAILBOX_1BitInjectTest
 *
 * @brief   Execute ECC MSS MAILBOX 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_MAILBOX_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS MAILBOX Single bit error inject : starting \r\n");

    /* Run one shot test for MSS MAILBOX 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x72000000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS MAILBOX Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_MAILBOX_1BitInjectTest() */
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
    void *ptr = (void *)&arg;

    if (retVal == 0) {
        /* Initialize ESM module */

        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);


            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }

	if (retVal == 0) {
		/*Init of R5F*/
		retVal = ECC_Test_R5F_init();

		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_R5F_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}
	if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest has failed... \r\n");
        }
    }
    if (retVal == 0) {

        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest has failed... \r\n");
        }
    }
    if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest has failed... \r\n");
        }
    }
	
	if (retVal == 0) {
        result = ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest has failed... \r\n");
        }
    }
	
	if (retVal == 0) {
		/*Init of MCAN*/
		retVal = ECC_Test_MCAN0_init();

		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_MCAN0_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}
    if (retVal == 0)
    {
        result = ECC_Test_run_MCAN0_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_1BitInjectTest has failed... \r\n");
            /*Low priority MCAN0 interrupt */
        }

        result = ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_N_ROWInjectTest has failed... \r\n");
            /*Low priority MCAN0 interrupt */
        }

        result = ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_Repeat_InjectTes thas failed... \r\n");
            /*Low priority MCAN0 interrupt */
        }

        result = ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest has failed... \r\n");
            /* Low priority MCAN0 interrupt */
        }
    }
    if (retVal == 0) {
		
        result = ECC_Test_run_MCAN0_2BitInjectTest();

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_2BitInjectTest has failed.... \r\n");
            /*High priority MCAN0 interrupt */
        }
		
        result = ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_N_ROWInjectTest has failed... \r\n");
			/*High priority MCAN0 interrupt */
		}

        result = ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_Repeat_InjectTest has failed... \r\n");
            /*High priority MCAN0 interrupt */
        }
        
		result = ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest has failed... \r\n");
            /* High priority MCAN0 interrupt */
        }
    }
	
	if (retVal == 0) {
		/*Init of MSS*/
		retVal = ECC_Test_MSS_init();

		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_MSS_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}
    if (retVal == 0)
    {
        result = ECC_Test_run_MSS_MAILBOX_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_MSS_MAILBOX_1BitInjectTest has failed... \r\n");
            /* High priority MCAN0 interrupt */
        }
	}
    if ( retVal == 0) {
        DebugP_log("\r\nECC SDL API tests: success\r\n");
    } else {
        DebugP_log("\r\nECC SDL API tests: failed\r\n");
    }

    return retVal;
}

/* ECC Function module test */
int32_t ECC_sdl_funcTest(void)
{
    int32_t testResult = 0;
	
	/* Initialize ECC callbacks within the Main ESM */
	testResult = SDL_ECC_initEsm(SDL_ESM_INST_MAIN_ESM0);
	if (testResult != SDL_PASS) {
		/* print error and quit */
		DebugP_log("\r\nECC_Test_init: Error initializing ECC callback for MSS ESM: result = %d\r\n", testResult);

		testResult = -1;
	} else {
		DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for MSS ESM \r\n");
	}

	if (testResult != 0)
    {
        DebugP_log("\r\nECC SDL ECC_Test_R5F_init: unsuccessful");
        return SDL_EFAIL;
    }
    /*Execute ECC sdl function test*/
    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
