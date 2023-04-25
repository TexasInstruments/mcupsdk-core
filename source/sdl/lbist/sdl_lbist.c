/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file  sdl_lbist.c
 *
 * @brief
 *  SDL implementation file for the lbist module.
 *
 *  \par
*/

#include <sdl/sdl_lbist.h>
#include <sdl/include/sdl_types.h>
#include <sdl/lbist/sdl_lbist_priv.h>

#include <sdl/dpl/sdl_dpl.h>

static int32_t SDL_LBIST_runTest(SDL_lbistRegs *pRegs, SDL_lbistInstInfo *pInfo)
{
    SDL_ErrType_t status = SDL_PASS;
    bool isLBISTRunning = (bool)false;

    /* Initialize done flag */
    pInfo->doneFlag = LBIST_NOT_DONE;

    /**-- Step 1: Configure LBIST --*/

    status = SDL_LBIST_programConfig(pRegs, &pInfo->LBISTConfig);

    /**-- Step 2: Run LBIST test --*/

    /**--- Step 2a: Enable Isolation ---*/
    if (status == SDL_PASS)
    {
        /* Call SDL API */
        status = SDL_LBIST_enableIsolation(pRegs);
        /**--- Step 2b: reset LBIST ---*/
        if (status == SDL_PASS)
        {
            status = SDL_LBIST_reset(pRegs);
            /**--- Step 2c: Enable Run BIST Mode ---*/
            if (status == SDL_PASS)
            {
                status = SDL_LBIST_enableRunBISTMode(pRegs);
            }
        }
    }

    /**--- Step 2d: Start LBIST ---*/
    if (status == SDL_PASS)
    {
        status = SDL_LBIST_start(pRegs);
        /**--- Step 2e: Check LBIST Running status ---*/
        if (status == SDL_PASS)
        {
            status = SDL_LBIST_isRunning(pRegs, &isLBISTRunning);
        }
    }
    return status;
}

/**
 * Design: PROC_SDL-1009,PROC_SDL-1010,PROC_SDL-1011,PROC_SDL-6237
 */

int32_t SDL_LBIST_selfTest(SDL_LBIST_inst instance, SDL_LBIST_testType testType)
{
  SDL_lbistRegs *pRegs;
  SDL_lbistInstInfo *pInfo;
  SDL_ErrType_t status = SDL_PASS;

  /* Get the LBIST Instance Info */
  pInfo = SDL_LBIST_getInstInfo((uint32_t)instance);

  if ((pInfo == NULL) ||
      ((testType != SDL_LBIST_TEST) && (testType != SDL_LBIST_TEST_RELEASE)))
  {
      status = SDL_EBADARGS;
  }

  if (status == SDL_PASS)
  {
      /* Get LBIST register space Pointer */
      pRegs = pInfo->pLBISTRegs;
  }

  if (status == SDL_PASS)
  {
      if (testType == SDL_LBIST_TEST)
      {
          status = SDL_LBIST_runTest(pRegs, pInfo);
      }
      else
      {
          status = SDL_LBIST_disableIsolation(pRegs);
      }
  }
  return status;
}

/**
 * Design: PROC_SDL-6238
 */
uint8_t SDL_LBIST_checkDone(SDL_LBIST_inst instance)
{
    SDL_lbistInstInfo *pInfo;

    /* Get the LBIST Instance Info */
    pInfo = SDL_LBIST_getInstInfo((uint32_t)instance);

    SDL_LBIST_eventHandler(&pInfo);

    return ((uint8_t)(pInfo->doneFlag));
}

/**
 * Design: PROC_SDL-6239
 */
int32_t SDL_LBIST_checkResult(SDL_LBIST_inst instance, bool *pResult)
{
    int32_t status = SDL_PASS;
    uint32_t calculatedMISR;
    uint32_t expectedMISR;
    SDL_lbistRegs *pRegs;
    SDL_lbistInstInfo *pInfo;

    /* Get the LBIST Instance Info */
    pInfo = SDL_LBIST_getInstInfo((uint32_t)instance);

    /* Get LBIST register space Pointer */
    /* Perform Address Translation. */
    pRegs = (SDL_lbistRegs *)SDL_DPL_addrTranslate((uint64_t)((pInfo->pLBISTRegs)), SDL_MCU_CTRL_MMR0_CFG0_SIZE);

    /**--- Step 2f: Get Signature of test ---*/
    status = SDL_LBIST_getMISR(pRegs, &calculatedMISR);

    /**--- Step 2g: Get Expected Signature ---*/
    if (status == SDL_PASS)
    {
        expectedMISR = pInfo->expectedMISR;
    }

    /**--- Step 2h: Clear Run BIST Mode ---*/
    if (status == SDL_PASS)
    {
        status = SDL_LBIST_clearRunBISTMode(pRegs);
    }

    /**--- Step 2i: Stop LBIST ---*/
    if (status == SDL_PASS)
    {
        status = SDL_LBIST_stop(pRegs);
    }

    /**--- Step 2j: Reset LBIST ---*/
    if (status == SDL_PASS)
    {
        status = SDL_LBIST_reset(pRegs);
    }

    /**--- Step 3: Check result of LBIST  ---*/
    if (status == SDL_PASS)
    {
        if (calculatedMISR != expectedMISR)
        {
            *pResult = (bool)false;
        }
        else
        {
            *pResult = (bool)true;
        }
    }
    return status;
}

/* Nothing past this point */
