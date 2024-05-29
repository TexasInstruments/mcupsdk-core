/*
 *   Copyright (c) 2022-2024 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include "sdlexample.h"


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This macro shows how many ESM events are configured*/
#define SDL_MCANA_MAX_MEM_SECTIONS      (1u)
#define SDL_EXAMPLE_ECC_RAM_ADDR        (0x52600000u) /* MCAN0 address */
#define SDL_EXAMPLE_ECC_AGGR            SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID          SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/*========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
volatile static bool esmEccError = false;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


/* Used by our ESM callback on ECC Errors for MCAN0 */
void ecc_clearESM(void)
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
 * ECC_Test_run_MCANA_1BitInjectTest
 *
 * Execute ECC MCANA  1 bit inject test
 *
 * param   None
 *
 * return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_1BitInjectTest(void)
{
    SDL_ErrType_t               result;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t           testLocationValue;

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    /* Run one shot test for MCANA  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result == SDL_PASS )
    {
      /* Access the memory where injection is expected - we must do this to trigger */
      testLocationValue = injectErrorConfig.pErrMem[0];
      testLocationValue++; /* this is purely to avoid misra unused error */
    }

    return result;
}/* End of ECC_Test_run_MCANA_1BitInjectTest() */

/*********************************************************************
 * ECC_Test_run_MCANA_2BitInjectTest
 *
 * Execute ECC MCANA  2 bit Inject test
 *
 * None
 *
 * return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCANA_2BitInjectTest(void)
{
    SDL_ErrType_t               result;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t           testLocationValue;

    /* Run one shot test for MCANA  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(SDL_EXAMPLE_ECC_RAM_ADDR);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_EXAMPLE_ECC_AGGR,
                                 SDL_EXAMPLE_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    if (result == SDL_PASS)
    {
      /* Access the memory where injection is expected - we must do this to trigger */
      testLocationValue = injectErrorConfig.pErrMem[0];
      testLocationValue++; /* this is purely to avoid unused variable error */
    }

    return result;
}/* End of ECC_Test_run_MCANA_2BitInjectTest() */

/*********************************************************************
 * ECC_test
 *
 * Execute ECC sdl function test

 * return  0 : Success; < 0 for failures
 **********************************************************************/
int32_t ECC_test(void)
{
    int32_t   result;
    int32_t   i =0;
    uint32_t  num_of_iterations = 10, wr_data = 0, addr = SDL_EXAMPLE_ECC_RAM_ADDR;
    uint32_t  rd_data=0;
    uint32_t  maxTimeOutMilliSeconds = 1000000000;
    uint32_t  timeOutCnt = 0;


    /* make sure flag used to detect ESM interrupt is cleared */
    esmEccError = false;

    /* run 1 Bit injection test */
    for(i=1;i<=num_of_iterations;i++)
    {
        wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
        SDL_REG32_WR(addr+i*16, wr_data);
    }

    result = ECC_Test_run_MCANA_1BitInjectTest();

    /* read the data where injection occured */
    for(i=1;i<=num_of_iterations; i++)
    {
        rd_data = SDL_REG32_RD(addr+i*16);
        rd_data = rd_data + 1u; /* avoids misra c unused */
    }

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

    /* reset esm error for next test  */
    esmEccError = false;

    /* run 2 Bit injection test */
    if (result == SDL_PASS)
    {
        for(i=1;i<=num_of_iterations;i++)
        {
          wr_data = (i)<<24 | (i)<<16 | (i)<<8 | i;
          SDL_REG32_WR(addr+i*16, wr_data);
        }

        result = ECC_Test_run_MCANA_2BitInjectTest();

        for(i=1;i<=num_of_iterations;i++)
        {
          rd_data = SDL_REG32_RD(addr+i*16);
          rd_data++; /* to avoid misra unused */
        }

        if (result == SDL_PASS)
        {
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    /* no interrupt received */
                    result = SDL_EFAIL;
                    break;
                }
            }  while (esmEccError == false);
        }

      /* reset esm error for next test  */
      esmEccError = false;
    }

    return result;
}



/* Nothing past this point */
