/*
 *   Copyright (c) Texas Instruments Incorporated 2025
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
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <sdl/sdl_esm.h>
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <stdio.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>

#if defined(SOC_AM64X) || defined (SOC_AM243X)
#include "soc/am64x_am243x/ecc_func.h"
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define SDL_ECC_CACHE_ARRAY_SIZE                        32*1024                     /* Size of the array in bytes (32KB) */
#define SDL_SINGLE_ERROR_CORRECTION                     (1)
#define SDL_DOUBLE_ERROR_DETECTION                      (2)

/* Starting and Ending RAM IDs of R5F D-Cache and I-Cache */

#define SDL_ECC_ITAG_RAM_START_ID                                           (0)
#define SDL_ECC_ITAG_RAM_END_ID                                             (3)
#define SDL_ECC_IDATA_RAM_START_ID                                          (4)
#define SDL_ECC_IDATA_RAM_END_ID                                            (7)
#define SDL_ECC_DTAG_RAM_START_ID                                           (8)
#define SDL_ECC_DTAG_RAM_END_ID                                             (11)
#define SDL_ECC_DDIRTY_RAM_START_ID                                         (12)
#define SDL_ECC_DDIRTY_RAM_END_ID                                           (12)
#define SDL_ECC_DDATA_RAM_START_ID                                          (13)
#define SDL_ECC_DDATA_RAM_END_ID                                            (20)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

volatile uint32_t ram_Id = 0;
static uint32_t gArg;
volatile uint8_t SDL_ECC_cache_ddata[SDL_ECC_CACHE_ARRAY_SIZE]__attribute__((section(".DATA_Cache.buffer")));

int32_t ECC_Memory_init(SDL_Test_EccConfig test_config);
void ecc_instructions_test_function(void);


extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* ========================================================================== */
/*                          Function Definitions                              */
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

void ECC_Test_exceptionInit(void)
{
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);
    return;
}

/*********************************************************************
* @fn      ECC_PMCR_export()
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/

void ECC_PMCR_export()
{
    asm("MOV R5, #0x0");                    /* Write zero value in R5 register */
    asm("MRC     P15,#0, R5, C9, C12,#0");  /* Read PMCR regitser */
    asm("ORR     R5, R5, #0x2");            /* Reset event counter */
    asm("MCR     P15,#0, R5, C9, C12,#0");  /* Write PMCR register */
    asm("MRC     P15,#0, R5, C9, C12,#0");  /* Read PMCR regitser */
    asm("ORR     R5, R5, #0x11");           /* enable all counters and export of events to the event bus */
    asm("MCR     P15,#0, R5, C9, C12,#0");  /* Write PMCR register */
    asm("MOV r4, #0x60");                   /* select some events to count by PMXEVTYPER register */
    asm("MCR p15,#0,r4,c9,c13,#1");         /* Write PMXEVTYPER register */
}

/*********************************************************************
* @fn      ECC_Memory_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/

int32_t ECC_Memory_init (SDL_Test_EccConfig test_config)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
    void *ptr = (void *)&gArg;

    /* Initialize exception handler */
    ECC_Test_exceptionInit();

    /* Enable the export functionality of R5F Performance Monitors Control Register for routing R5F ECC error events to ESM/ Pulsar registers */
    ECC_PMCR_export();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

    if (retValue == 0) {
		uknownErr = true;
        /* Initialize ESM module */
        result = SDL_ESM_init(ECC_Test_config.instance, ECC_Test_config.pConfig, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Memory_init: Error initializing ESM: result = %d\r\n", result);
            retValue = -1;
        } else {
            DebugP_log("\r\nESM_Memory_init: Init ESM complete \r\n");
			uknownErr = false;
			esmError = false;
        }
    }
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(ECC_Test_config.eccMemType, ECC_Test_config.initConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Memory_init: Error initializing R5FSS0 CORE0 ECC: result = %d\r\n", result);
            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Memory_init: R5FSS0 CORE0 ECC initialization is completed \r\n");
        }
    }
    return retValue;
}/* End of ECC_Memory_init() */

/*********************************************************************
 * @fn      ECC_I_Cache_InjectTest
 *
 * @brief   Execute ECC I-Cache inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_I_Cache_InjectTest(SDL_ECC_MemType eccMemType,uint32_t ram_Id,SDL_ECC_InjectErrorType errorType)
{
    volatile uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    SDL_ErrType_t result;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    CacheP_wbInvAll(CacheP_TYPE_L1P);
    ecc_instructions_test_function();

	DebugP_log("\r\nI_Cache 1-bit Error Injection : Starting for RAM ID = %d\r\n",ram_Id);

    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);         /* Note the address is relative to start of ram */
    injectErrorConfig.flipBitMask = 0x10;                   /* Run one shot test for I_Cache 1 bit error */

    result = SDL_ECC_injectError( eccMemType,
                                 (SDL_ECC_MemSubType)ram_Id,
                                  errorType,
                                  &injectErrorConfig);

    if (result != SDL_PASS ) {
        return (int32_t)result;
    }

    ecc_instructions_test_function();
    asm("NOP");                                             /* ideal cycle needed for error injection in R5F cache */

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
        DebugP_log("\r\nInjected 1-bit error and got ESM Interrupt for ram_Id = %d\r\n", ram_Id);
        /* Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation */
        esmError = false;
    }
    if (result != SDL_PASS) {
        DebugP_log("\r\nICache 1-bit error injection failed...\r\n");
    }

    return (int32_t)result;
}/* End of ECC_I_Cache_InjectTest() */

/*********************************************************************
 * @fn      ECC_D_Cache_InjectTest
 *
 * @brief   Execute ECC D-Cache inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_D_Cache_InjectTest(SDL_ECC_MemType eccMemType,uint32_t ram_Id,SDL_ECC_InjectErrorType errorType,uint32_t num_bits)
{
    volatile uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    SDL_ErrType_t result;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

    CacheP_wbInvAll(CacheP_TYPE_L1D);
    for(uint32_t cache_byte_count = 0u; cache_byte_count < SDL_ECC_CACHE_ARRAY_SIZE; cache_byte_count++)
    {
        SDL_ECC_cache_ddata[cache_byte_count] = (cache_byte_count%256)+1;
    }

	DebugP_log("\r\nD_Cache Error Injection : Starting for RAM ID = %d\r\n",ram_Id);

    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);             /* Note the address is relative to start of ram */
    if(num_bits == 1)
        injectErrorConfig.flipBitMask = 0x10;                   /* Run test for D_Cache 1 bit error */
    else
        injectErrorConfig.flipBitMask = 0x03;                   /* Run test for D_Cache 2 bit error */

    result = SDL_ECC_injectError( eccMemType,
                                 (SDL_ECC_MemSubType)ram_Id,
                                  errorType,
                                  &injectErrorConfig);

    if (result != SDL_PASS ) {
        return (int32_t)result;
    }

    for(uint32_t cache_byte_count = 0u; cache_byte_count < SDL_ECC_CACHE_ARRAY_SIZE; cache_byte_count++)
    {
        SDL_ECC_cache_ddata[cache_byte_count] = SDL_ECC_cache_ddata[cache_byte_count];
    }
    asm("NOP");                                                 /* ideal cycle needed for error injection in R5F cache */

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
        DebugP_log("\r\nInjected error and got ESM Interrupt for ram_Id = %d\r\n", ram_Id);
        /* Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation */
        esmError = false;
    }
    if (result != SDL_PASS) {
        DebugP_log("\r\nDCache error injection failed...\r\n");
    }

    return (int32_t)result;
}/* End of ECC_D_Cache_InjectTest() */

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
    volatile int32_t result = 0;

    SDL_ECC_MemSubType *subMemTypeList = ECC_Test_config.initConfig->pMemSubTypeList;    /**< List of R5F Cache Ram IDs of the aggregator */

    DebugP_log("\r\nECC R5F Cache test started for aggregator = %s\r\n", ECC_Test_config.aggrName);

    DebugP_log("\r\nStarting Tests for Itag - single error correction\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_ITAG_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_ITAG_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_I_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT);
    }

    DebugP_log("\r\nStarting Tests for IData - single error correction\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_IDATA_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_IDATA_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_I_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT);
    }

    DebugP_log("\r\nStarting Tests for Dtag - single error correction\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DTAG_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DTAG_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,SDL_SINGLE_ERROR_CORRECTION);
    }

    DebugP_log("\r\nStarting Tests for DDirty cache - single error correction\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DDIRTY_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DDIRTY_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,SDL_SINGLE_ERROR_CORRECTION);
    }

    DebugP_log("\r\nStarting Tests for Ddata cache - single error correction\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DDATA_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DDATA_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,SDL_SINGLE_ERROR_CORRECTION);
    }

    DebugP_log("\r\nStarting Tests for Dtag cache - double error detection\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DTAG_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DTAG_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,SDL_DOUBLE_ERROR_DETECTION);
    }

    DebugP_log("\r\nStarting Tests for Ddirty cache - double error detection\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DDIRTY_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DDIRTY_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,SDL_DOUBLE_ERROR_DETECTION);
    }

    DebugP_log("\r\nStarting Tests for Ddata cache - double error detection\r\n");
    for (ram_Id = subMemTypeList[SDL_ECC_DDATA_RAM_START_ID] ; ram_Id <= subMemTypeList[SDL_ECC_DDATA_RAM_END_ID]; ram_Id++)
    {
        result |= ECC_D_Cache_InjectTest(ECC_Test_config.eccMemType,ram_Id,SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,SDL_DOUBLE_ERROR_DETECTION);
    }

    return result;
}/* End of ECC_sdlFuncTest() */

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
}/* End of sdlApp_dplInit() */

/* ECC Function module test */
int32_t ECC_funcTest(void)
{
    int32_t testResult = 0;

    /* Initializing the DPL */
    sdlApp_dplInit();

    /* Initializing required modules */
    testResult = ECC_Memory_init(ECC_Test_config);

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\nECC R5 Cache tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    /* Execute ECC sdl function test */
    testResult = ECC_sdlFuncTest();

    return (testResult);
}/* End of ECC_funcTest() */

/* Nothing past this point */
