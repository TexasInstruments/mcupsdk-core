/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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
#include <dpl_interface.h>

#if defined(SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM263PX)
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay for 1us*/
#define DELAY 1

/* This macro shows how many ESM events are configured*/
#define SDL_ESM_MAX_MSS_EXAMPLE_AGGR                (2U)
#define SDL_MSS_L2_MAX_MEM_SECTIONS                 (1U)

#if defined(SOC_AM263X) || defined(SOC_AM263PX)
#define SDL_EXAMPLE_ECC_RAM_ADDR                    (0x70000A00U) /*MSS_L2_SLV0 address*/
#define SDL_EXAMPLE_ECC_AGGR                        SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                      SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID

#define SDL_MSS_L2_MEM_INIT_ADDR                    (0x50D00240U)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR               (0x50D00244U)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR             (0x53000020U)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                (0x0CU) /*Bank 3 and 2*/
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static SDL_ECC_MemSubType ECC_Test_MSS_L2_subMemTypeList[SDL_MSS_L2_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSS_L2_ECCInitConfig =
{
    .numRams = SDL_MSS_L2_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSS_L2_subMemTypeList[0]),
    /**< Sub type list  */
};

/* ========================================================================== */
/*                  Function Declarations                                     */
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

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MSS_L2_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MSS L2 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MSS L2 ECC initialization is completed \r\n");
        }
    }
    return retValue;
}

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2RAMB_1BitInjectTest
 *
 * @brief   Execute ECC MSS L2 RAMB 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2RAMB_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nMSS L2 RAMB Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for MSS L2 RAMB 1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        ;
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2RAMB_1BitInjectTest() */

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

    if (retVal == 0)
    {
        /*Inject ECC Single bit error*/
        result = ECC_Test_run_MSS_L2RAMB_1BitInjectTest();

        if (result != SDL_PASS) 
        {
            retVal = -1;
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

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /*Clearing any old interrupt presented*/
    SDL_REG32_WR(SDL_ECC_AGGR_ERROR_STATUS1_ADDR, 0xF0Fu);


    /*Initializing required modules*/
    testResult = ECC_Example_init();

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\nECC Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    return (testResult);
}

/* Nothing past this point */
