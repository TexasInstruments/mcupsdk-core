/* Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  \file     main.c
 *
 *  \brief    This file contains STC example code.
 *
 *  \details  STC app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <sdl/sdl_stc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "sdlexample.h"


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*
 *  \brief global variable for holding data buffer.
*/
static const SDL_STC_Inst test_case[]={SDL_STC_INST_MAINR5F0, SDL_STC_INST_MAINR5F1};

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

/*
** int32_t STC_test(int32_t coreNumber)
**
** Starts a self test on the core specified. .
*/
int32_t STC_test(int32_t coreNumber)
{
    int32_t sdlResult=SDL_PASS;
    SDL_STC_Config configVal;

    /* for R5F core the default configuration */
    SDL_STC_Config *pConfig=&configVal;
    pConfig->intervalNum=                              STC_MSS_INTERVAL_NUM;
    pConfig->modeConfig.lpScanMode=                    STC_MSS_LP_SCAN_MODE;
    pConfig->modeConfig.codecSpreadMode=               STC_MSS_CODEC_SPREAD_MODE;
    pConfig->modeConfig.capIdleCycle=                  STC_MSS_CAP_IDLE_CYCLE;
    pConfig->modeConfig.scanEnHighCap_idleCycle=       STC_MSS_SCANEN_HIGH_CAP_IDLE_CYCLE;
    pConfig->maxRunTime=                               STC_MSS_MAX_RUN_TIME;
    pConfig->clkDiv=                                   STC_MSS_CLK_DIV;
    pConfig->romStartAddress=                          STC_ROM_START_ADDRESS;
    pConfig->pRomStartAdd=                             STC_pROM_START_ADDRESS;

    /* Two type of test cases are supported\
        1.SDL_STC_TEST       - Normal test.
        2.SDL_STC_NEG_TEST   - Deliberatly fail. Useful for testing.
    */
    sdlResult=   SDL_STC_selfTest(test_case[coreNumber], SDL_STC_TEST, pConfig);

    return sdlResult;
}

/*
**void STC_run(void *args)
**
** Run a self test on each of the R5 cores and store the results.
*/
void STC_run(void *args)
{
    /* disable IRQ */
    HwiP_disable();

    static int32_t countInst=1,i;

    /* Run the tests */
    for (i=countInst; i>=0 ; i--)
    {
        sdlstats.stcResult[i] =  SDL_STC_getStatus(test_case[i]);
        if (sdlstats.stcResult[i] == SDL_STC_NOT_RUN)
        {
            /* test not run yet, run it */
          sdlstats.stcResult[i] =  STC_test(i);
        }
    }

    /* Get the results */
    for (i=countInst; i>=0 ; i--)
    {
        sdlstats.stcResult[i] =  SDL_STC_getStatus(test_case[i]);
    }

  /* Re-enable IRQ */
  HwiP_enable();

  return;
}

/* Nothing past this point */
