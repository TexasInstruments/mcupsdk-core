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
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/**
*	\brief  Developer can enable or disable below macros for the testing off
*           ICSSM different banks
*           Only one macro should be enabled at one time.
**/
#define SDL_ICSSM_DRAM0								(1U)
#define	SDL_ICSSM_DRAM1								(0U)
#define	SDL_ICSSM_PR1_PDSP0_IRAM					(0U)
#define	SDL_ICSSM_PR1_PDSP1_IRAM					(0U)
#define SDL_ICSSM_RAM								(0U)

#define SDL_ICSSM_MAX_MEM_SECTIONS           		(1u)

#if SDL_ICSSM_DRAM0
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR                    (0x48000000u) /* ICSSM DRAM0 RAM address */
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID
#endif

#if SDL_ICSSM_DRAM1
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR                    (0x48002000u) /* ICSSM DRAM1 RAM address */
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID
#endif

#if SDL_ICSSM_PR1_PDSP0_IRAM
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR                    (0x48034000u) /* ICSSM PR1 PDSP0 IRAM RAM address */
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID
#endif

#if SDL_ICSSM_PR1_PDSP1_IRAM
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR                    (0x48038000u) /* ICSSM PR1 PDSP1 IRAM RAM address */
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID
#endif

#if SDL_ICSSM_RAM
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR                    (0x48010000u) /* ICSSM RAM RAM address */
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

volatile static bool esmEccError = false;


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ECC_Example_init function */
int32_t ECC_Example_init (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ecc_icssm_clearESM(void)
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

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR);

    /* Run one shot test for ICSSM 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_ICSSM_AGGR,
                                 SDL_EXAMPLE_ECC_ICSSM_RAM_ID,
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
}/* End of ECC_Test_run_ICSSM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    /* Run one shot test for ICSSM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_ICSSM_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_ICSSM_AGGR,
                                 SDL_EXAMPLE_ECC_ICSSM_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        testLocationValue++; /* this is purely to avoid misra unused error */
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t ECC_ICSSM_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;

    if (retVal == 0)
    {
        result = ECC_Test_run_ICSSM_2BitInjectTest();
        if (result == SDL_PASS)
        {
            /* waiting for ESM interrupt */
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
            /* UC-1 Low priority ICSSM interrupt */
        }
    }

	if (retVal == 0) {
        result = ECC_Test_run_ICSSM_1BitInjectTest();
        if (result == SDL_PASS)
        {
            /* Waiting for ESM interrupt */
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
			/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
            esmEccError = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            /* UC-2 High priority ICSSM interrupt */
        }
    }

    return retVal;
}



/* ECC Function module test */
int32_t ECC_ICSSM_test(void)
{
    int32_t testResult = 0;

    /*Execute ECC sdl function test*/
    testResult = ECC_ICSSM_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
