/*
 *   Copyright (c) Texas Instruments Incorporated 2024
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
#include "sdlexample.h"
#include <sdl/dpl/sdl_dpl.h>

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
#define SDL_ESM_MAX_MSS_EXAMPLE_AGGR                (2u)
#define SDL_MSS_L2_MAX_MEM_SECTIONS                 (1u)

#if defined(SOC_AM263X) || defined(SOC_AM263PX)
#define SDL_EXAMPLE_ECC_MSSL2_RAM_ADDR                    (0x70100008u) /*MSS_L2_SLV2 address*/
#define SDL_EXAMPLE_ECC_MSSL2_AGGR                        SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_MSSL2_RAM_ID                      SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID

#define SDL_MSS_L2_MEM_INIT_ADDR                    (0x50D00240u)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR               (0x50D00244u)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR             (0x53000020u)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                (0xcu) /*Bank 3*/
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile static bool esmEccError = false;

void ecc_mssl2_clearESM(void)
{
  SDL_ECC_MemType     eccmemtype;
  SDL_Ecc_AggrIntrSrc eccIntrSrc;
  SDL_ECC_ErrorInfo_t eccErrorInfo;

  SDL_ECC_getESMErrorInfo(sdlstats.esm.esmInst, sdlstats.esm.intSrc, &eccmemtype, &eccIntrSrc);
  SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

  sdlstats.ecc.eccMemtype   = eccmemtype;
  sdlstats.ecc.eccIntrSrc   = eccIntrSrc;
  sdlstats.ecc.eccErrorInfo = eccErrorInfo;

  if (eccErrorInfo.injectBitErrCnt != 0)
  {
      SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
  }
  else
  {
      SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
  }

  SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

  esmEccError = true;

  return;
}
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


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
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_MSSL2_RAM_ADDR);

    /* Run one shot test for MSS L2 RAMB 1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_MSSL2_AGGR,
                                 SDL_EXAMPLE_ECC_MSSL2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        testLocationValue++; /* this is purely to avoid misra unused error */
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2RAMB_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2RAMB_2BitInjectTest
 *
 * @brief   Execute ECC MSS L2 RAMB 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2RAMB_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Run one shot test for MSS L2 RAMB 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_MSSL2_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x30002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_MSSL2_AGGR,
                                 SDL_EXAMPLE_ECC_MSSL2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    testLocationValue++; /* this is purely to avoid misra unused error */

    if (result != SDL_PASS ) {
       retVal = -1;
    }
    return retVal;
}/* End of ECC_Test_run_MSS_L2RAMB_2BitInjectTest() */

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
        /*Inject ECC Single bit error*/
        result = ECC_Test_run_MSS_L2RAMB_1BitInjectTest();

        if (result == SDL_PASS)
        {
            do
            {
                timeOutCnt += 1;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmEccError == false);
        }
        if(result == SDL_PASS){
            esmEccError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            /* UC-1 Low priority MSS L2 interrupt */
        }
    }
    if (retVal == 0) {


#if  defined (SOC_AM263X) || defined(SOC_AM263PX)
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_EXAMPLE_ECC_MSSL2_AGGR, SDL_EXAMPLE_ECC_MSSL2_RAM_ID);
#endif
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        esmEccError = false;

        /*Inject ECC Double bit error*/
        result = ECC_Test_run_MSS_L2RAMB_2BitInjectTest();

        if (result == SDL_PASS)
        {
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    result = SDL_EFAIL;
                    break;
                }
            } while (esmEccError == false);
        }
        if(result == SDL_PASS){
            esmEccError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            /* UC-2 High priority MSS L2 interrupt */
        }
    }

    return retVal;
}



/* ECC Function module test */
int32_t ECC_MSSL2_test(void)
{
    int32_t testResult = 0;

#if defined(SOC_AM263X) || defined(SOC_AM263PX)
    /* Clear Done memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, 0xfu);
#endif

    /* Initialization of MSS L2 memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    while(SDL_REG32_RD(SDL_MSS_L2_MEM_INIT_DONE_ADDR)!=SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /*Clearing any old interrupt presented*/
    SDL_REG32_WR(SDL_ECC_AGGR_ERROR_STATUS1_ADDR, 0xF0Fu);


    if (testResult != SDL_PASS)
    {
        return SDL_EFAIL;
    }

    /*Execute ECC sdl function test*/
    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
