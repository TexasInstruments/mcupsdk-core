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
#include <kernel/dpl/DebugP.h>
#include "ecc_main.h"
#include <sdl/dpl/sdl_dpl.h>

#if defined(SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM263PX)
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>
#endif
#if defined (SOC_AM261X)
#include <sdl/include/am261x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM273X)
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay for 1us*/
#define DELAY 1

/* This macro shows how many ESM events are configured*/
#define SDL_MCANA_MAX_MEM_SECTIONS                  (1u)
#if defined(SOC_AM273X) || defined(SOC_AWR294X)

#define SDL_ESM_MAX_MCANA_EXAMPLE_AGGR              (2u)
#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_GROUP_NUM_2                        (2U)
#define SDL_INTR_GROUP_NUM_3                        (3U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x02040000u) /* MCANA RAM address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_MSS_MCANA_ECC
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID
#define SDL_ECC_AGGR_SEC_ENABLE_SET_REG0_ADDR       (0x02F7F880u)
#endif

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)

#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x52600000u) /* MCAN0 address */
#define SDL_EXAMPLE_ECC_AGGR                        SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID

#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static SDL_ECC_MemSubType ECC_Test_MCANA_subMemTypeList[SDL_MCANA_MAX_MEM_SECTIONS] =
{
    SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCANA_ECCInitConfig =
{
    .numRams = SDL_MCANA_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCANA_subMemTypeList[0]),
    /**< Sub type list  */
};

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)

static uint32_t ESMarg;

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x0000000cu, 0x00000000u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events enable: except clkstop events for unused clocks
     *   and PCIE events */
    /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x00000008u, 0x000000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x0000000cu, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
#if  defined(SOC_AM263PX) || defined (SOC_AM261X)
    .enableCriticalBitmap = {0x0000000cu, 0x00000000u, 0x00000000u, 0x00000000u,
                             0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     .criticalInterruptDelayCounter = 0u,
#endif
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *ESMarg);

#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *ESMarg);

/* Event BitMap for ECC ESM callback for MCANA*/
SDL_ESM_NotifyParams ECC_TestparamsMCANA[SDL_ESM_MAX_MCANA_EXAMPLE_AGGR] =
{
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
};
#endif
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
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    void *ptr = (void *)&ESMarg;
#endif
    SDL_ErrType_t result;

    if (retValue == 0) {
        /* Initialize ECC Memory */
        result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCANA ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: Initialize of MCANA ECC Memory is complete \r\n");
        }
    }
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    if (retValue == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (retValue == SDL_PASS)
        {
            DebugP_log("\r\nESM_Test_init: Init MCANA ESM complete \r\n");
        }
        else {
            DebugP_log("\r\nECC_Example_init: Error initializing ESM: result = %d\r\n", result);
            retValue = -1;
        }
    }
#endif

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MCANA_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCANA ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCANA ECC initialization is completed \r\n");
        }
    }
    return retValue;
}

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

	DebugP_log("\r\nMCANA Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for MCANA  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCANA Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
        testLocationValue++; /* this is purely to avoid misra unused error */
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_1BitInjectTest() */

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

	DebugP_log("\r\nMCANA double bit error inject: starting \r\n");

    /* Run one shot test for MCANA  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nMCANA Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p\r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
         testLocationValue++; /* this is purely to avoid misra unused error */
    }

    return retVal;
}/* End of ECC_Test_run_MCANA_2BitInjectTest() */

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
    int32_t retVal = 0, i =0;
    uint32_t num_of_iterations = 10, wr_data = 0, addr = SDL_EXAMPLE_ECC_RAM_ADDR, rd_data = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;

    if (retVal == 0)
    {
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMCANA[0],NULL,NULL);
		if (result == SDL_PASS)
        {
            DebugP_log("\r\nESM_Test_init: Init MCANA ESM single bit complete\r\n");
        }
        else {
            DebugP_log("\r\nECC_Example_init: Error initializing ESM for single bit: result = %d\r\n", result);
            result = SDL_EFAIL;
        }
#endif
        for(i=1;i<=num_of_iterations;i++){
            wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
            SDL_REG32_WR(addr+i*16, wr_data);
        }
        result = ECC_Test_run_MCANA_1BitInjectTest();

        for(i=1;i<=num_of_iterations;i++){
            rd_data = SDL_REG32_RD(addr+i*16);
            DebugP_log("\r\nRead data =  0x%p\r\n",rd_data);
        }


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
            DebugP_log("\r\nUC-1: Injected 1-bit error and got ESM Interrupt \r\n");
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nESM_ECC_Example_run: UC-1 has failed...\r\n");
            /* UC-1 Low priority MCANA interrupt */
        }
    }
    if (retVal == 0) {
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
        /* Initialize ESM module */
		result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_TestparamsMCANA[1],NULL,NULL);
		if (result == SDL_PASS)
        {
            DebugP_log("\r\nESM_Test_init: Init MCANA ESM double bit complete \r\n");
        }
        else {
            DebugP_log("\r\nECC_Example_init: Error initializing ESM for double bit: result = %d\r\n", result);
            result = SDL_EFAIL;
        }
#endif
        for(i=1;i<=num_of_iterations;i++){
            wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
            SDL_REG32_WR(addr+i*16, wr_data);
        }

        result = ECC_Test_run_MCANA_2BitInjectTest();

        for(i=1;i<=num_of_iterations;i++){
            rd_data = SDL_REG32_RD(addr+i*16);
            DebugP_log("\r\nRead data =  0x%p\r\n",rd_data);
        }

        if (result == SDL_PASS)
        {
            DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
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
            DebugP_log("\r\nUC-2: Injected 2-bit error and got ESM Interrupt\r\n");
            esmError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nESM_ECC_Example_run: UC-2 has failed....\r\n");
            /* UC-2 High priority MCANA interrupt */
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
