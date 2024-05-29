/*
 *  Copyright (c) 2021-2024 Texas Instruments Incorporated
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
 *  \file     dcc_uc1.c
 *
 *  \brief    This file contains DCC Example test code.
 *
 *  \details  DCC tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "dcc_uc1.h"
#include <sdl/dpl/sdl_dpl.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/am263x/sdlr_intr_r5fss0_core0.h>
#include "sdlexample.h"

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
volatile  SDL_DCC_Inst gCurDccInst;

#define NUM_USE_CASES          (5U)

static DCC_TEST_UseCase DCC_Test_UseCaseArray[NUM_USE_CASES] =
{
    /* Single Shot - No error */
    {
        "XTAL_CLK",
        "SYSCLK0",
        SDL_DCC_INST_MSS_DCCA,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC0,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT,
        SDL_R5FSS0_CORE0_INTR_DCC0_DONE,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "XTAL_CLK",
        "SYSCLK0",
        SDL_DCC_INST_MSS_DCCA,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC0,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Single Shot - No error */
    {
        "RCCLK10M",
        "SYSCLK0",
        SDL_DCC_INST_MSS_DCCA,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        10000, /* 10 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCKSRC0,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT,
        SDL_R5FSS0_CORE0_INTR_DCC0_DONE,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "XTAL_CLK",
        "RCCLK32K",
        SDL_DCC_INST_MSS_DCCC,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HSOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC0,
        200000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "RCCLK10M",
        "XTAL_CLK",
        SDL_DCC_INST_MSS_DCCA,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        10000, /* 10 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCKSRC3,
        25000, /* 25 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
};


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/


static void SDL_DCCAppGetClkRatio(uint32_t  refClkFreq,
                              uint32_t  testClkFreq,
                              uint32_t *refClkRatioNum,
                              uint32_t *testClkRatioNum)
{
    uint32_t loopCnt, hcf = 1U;

    for (loopCnt = 1;
         (loopCnt <= refClkFreq) && (loopCnt <= testClkFreq);
         loopCnt++)
    {
        if ((refClkFreq % loopCnt == 0) && (testClkFreq % loopCnt == 0))
        {
            hcf = loopCnt;
        }
    }
    *refClkRatioNum  = (refClkFreq / hcf);
    *testClkRatioNum = (testClkFreq / hcf);
}

static void SDL_DCCAppSetSeedVals(uint32_t       refClkFreq,
                                  uint32_t       testClkFreq,
                                  uint32_t       refClkRatioNum,
                                  uint32_t       testClkRatioNum,
                                  uint32_t       drfitPer,
                                  SDL_DCC_config *configParams)
{
    uint32_t maxFreqKHz, maxCntLimit;
    uint32_t maxRefCnt, minRefCnt;
    uint64_t mulVar;

    /* Find maximum frequency between test and reference clock */
    if (refClkFreq > testClkFreq)
    {
        maxFreqKHz  = refClkFreq;
        maxCntLimit = APP_DCC_SRC0_MAX_VAL;
    }
    else
    {
        maxFreqKHz  = testClkFreq;
        maxCntLimit = APP_DCC_SRC1_MAX_VAL;
    }
    /* Calculate seed values for 0% drift */
    if (maxFreqKHz == refClkFreq)
    {
        configParams->clk0Seed = maxCntLimit / refClkRatioNum;
        configParams->clk0Seed = configParams->clk0Seed * refClkRatioNum;
        mulVar = ((uint64_t) (configParams->clk0Seed) *
                  (uint32_t) (testClkRatioNum));
        configParams->clk1Seed   = (uint32_t) (mulVar / refClkRatioNum);
        configParams->clk0ValidSeed = refClkRatioNum;
    }
    else
    {
        configParams->clk1Seed = maxCntLimit / testClkRatioNum;
        configParams->clk1Seed = configParams->clk1Seed * testClkRatioNum;
        mulVar = ((uint64_t) (configParams->clk1Seed) *
                  (uint32_t) (refClkRatioNum));
        configParams->clk0Seed   = (uint32_t) (mulVar / testClkRatioNum);
        configParams->clk0ValidSeed = 1U;
    }
    /* Applying allowed drift */
    if (((APP_DCC_SRC0_MAX_VAL + APP_DCC_SRC0_VALID_MAX_VAL) <
         (configParams->clk0Seed * (100U + drfitPer) / 100U)))
    {
        /* Seed values with drift exceeds maximum range */
        DebugP_log("[SDL-DCC] Seed values with drift exceeds allowed range.Example to run with 0% allowed drift. \r\n");
    }
    else if (100U < drfitPer)
    {
        /* Error percentage is greater than 100 */
        DebugP_log("[SDL-DCC] Warning Wrong drift %,Not applying drift. Example will run with 0% drift. \r\n");
    }
    else
    {
        maxRefCnt = (configParams->clk0Seed * (100U + drfitPer) / 100U);
        minRefCnt = (configParams->clk0Seed * (100U - drfitPer) / 100U);
        if (APP_DCC_SRC0_VALID_MAX_VAL < (maxRefCnt - minRefCnt))
        {
            DebugP_log("[SDL-DCC] Warning Seed value for valid count exceeds allowed range. App will run with 0% drift. \r\n");
        }
        else
        {
            if (maxRefCnt == minRefCnt)
            {
                configParams->clk0ValidSeed = 1U;
            }
            else
            {
                configParams->clk0Seed   = minRefCnt;
                configParams->clk0ValidSeed = (maxRefCnt - minRefCnt);
            }
        }
    }
}

static void SDL_DCCAppDoneIntrISR(void *arg)
{
    SDL_DCC_clearIntr(gCurDccInst, SDL_DCC_INTERRUPT_DONE);
    sdlstats.dcc.doneIsrFlag  = 1U;
}

static int32_t SDL_DCCAppRegisterIsr(uint32_t uc, pSDL_DPL_HwipHandle *handle)
{
    int32_t retVal = SDL_EFAIL;
    SDL_DPL_HwipParams intrParams;

    intrParams.intNum      = DCC_Test_UseCaseArray[uc].intNum;
    intrParams.callback    = &SDL_DCCAppDoneIntrISR;
    intrParams.callbackArg = 0x0;

    /* Register call back function for DCC Done interrupt */
    retVal = SDL_DPL_registerInterrupt(&intrParams, handle);

    return (retVal);
}

static void SDL_DCCAppDeRegisterIsr(pSDL_DPL_HwipHandle handle)
{
    SDL_DPL_deregisterInterrupt(handle);
}

/**
 *   This function waits infinitely for DCC done interrupt
 *
 * returns  SDL_PASS on occurrence DCC completion and no error.
 *          SDL_EFAIL otherwise.
 */
static int32_t SDL_DCCAppWaitForCompletion(void)
{
    int32_t retVal;

    /* Wait for completion interrupt / or error flag*/
    while ((DCC_NO_INTERRUPT == sdlstats.dcc.doneIsrFlag) && (DCC_NO_INTERRUPT == sdlstats.dcc.isrFlag));

    /* Ensure no error */
    if (sdlstats.dcc.isrFlag == DCC_INTERRUPT)
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

/* This dunctions is called from our ESM callabck when we detect a DCC event */
void dcc_clearESM(void)
{
    sdlstats.dcc.isrFlag = DCC_INTERRUPT;
    /* Clear DCC event */
    SDL_DCC_clearIntr(gCurDccInst, SDL_DCC_INTERRUPT_ERR);
    return ;
}

/* runs our diagnsotics for DCC */
int32_t DCC_test (void)
{
    /* Declarations of variables */
    int32_t         retVal;
    uint32_t        clk0Freq, clk1Freq, refClkRatioNum, testClkRatioNum;
    SDL_DCC_config  configParams;
    uint32_t        i;


    sdlstats.dcc.isrFlag      = 0U;
    sdlstats.dcc.doneIsrFlag  = 0U;

    for (i = 0; i < NUM_USE_CASES; i++)
    {
        gCurDccInst = DCC_Test_UseCaseArray[i].dccInst;
        clk0Freq = DCC_Test_UseCaseArray[i].clk0Freq;
        clk1Freq = DCC_Test_UseCaseArray[i].clk1Freq;

        if (DCC_Test_UseCaseArray[i].errorTest == 0x1)
        {
            /* Deliberately change the Reference Clock to 2 times to
             * introduce the error in the clock ratio
             */
            clk1Freq *= 2;
        }


            /* Get clock ratio */
            SDL_DCCAppGetClkRatio(clk0Freq,
                                  clk1Freq,
                                  &refClkRatioNum,
                                  &testClkRatioNum);

            configParams.mode    = DCC_Test_UseCaseArray[i].mode;
            configParams.clk0Src = DCC_Test_UseCaseArray[i].clk0;
            configParams.clk1Src = DCC_Test_UseCaseArray[i].clk1;

            /* Get the seed values for given clock selections and allowed drift */
            SDL_DCCAppSetSeedVals(clk0Freq,
                                  clk1Freq,
                                  refClkRatioNum,
                                  testClkRatioNum,
                                  APP_DCC_TEST_CLOCK_SRC_1_DRIFT,
                                  &configParams);


            retVal = SDL_DCC_configure(DCC_Test_UseCaseArray[i].dccInst, &configParams);

            if (SDL_PASS == retVal)
            {
                retVal = SDL_DCC_verifyConfig(DCC_Test_UseCaseArray[i].dccInst, &configParams);
            }
            else
            {
                retVal = SDL_EFAIL;
            }

            if (retVal == SDL_PASS)
            {
                /* Enable ERROR interrupt */
                SDL_DCC_enableIntr(DCC_Test_UseCaseArray[i].dccInst, SDL_DCC_INTERRUPT_ERR);

                /*
		             * Check for single-shot mode and enable interrupt for Done notification
                 * then wait for completion.
                 */
                if (DCC_Test_UseCaseArray[i].mode != SDL_DCC_MODE_CONTINUOUS)
                {
                    pSDL_DPL_HwipHandle handle;

                    SDL_DCCAppRegisterIsr(i, &handle);

                    /* Enable DONE interrupt(only for single shot mode) */
                    SDL_DCC_enableIntr(DCC_Test_UseCaseArray[i].dccInst, SDL_DCC_INTERRUPT_DONE);

                    SDL_DCC_enable(DCC_Test_UseCaseArray[i].dccInst);

                    if (SDL_PASS != SDL_DCCAppWaitForCompletion())
                    {
                        retVal = SDL_EFAIL;
                    }
                    SDL_DCCAppDeRegisterIsr(handle);
                    sdlstats.dcc.isrFlag = DCC_NO_INTERRUPT;
                    sdlstats.dcc.doneIsrFlag = 0x0;
                }
                else
                {
                    SDL_DCC_enable(DCC_Test_UseCaseArray[i].dccInst);
                    /* Wait for error notification */
                    volatile int32_t j = 0;
                    /* Wait for the ESM interrupt to report the error */
                    do {
                        j++;
                        if (j > 0x0FFFFFF)
                        {
                            /* Timeout for the wait */
                            break;
                        }

                    } while (sdlstats.dcc.isrFlag == DCC_NO_INTERRUPT);

                    if (sdlstats.dcc.isrFlag == DCC_INTERRUPT)
                    {
                        if (DCC_Test_UseCaseArray[i].errorTest == 0x0)
                        {
                            /* error event not expectedc */
                            retVal = SDL_EFAIL;
                        }
                    }
                    else
                    {
                        if (DCC_Test_UseCaseArray[i].errorTest == 0x1)
                        {
                            /* Could not generate Error interrupt */
                            retVal = SDL_EFAIL;
                        }
                    }
                    sdlstats.dcc.isrFlag = DCC_NO_INTERRUPT;
                }
                SDL_DCC_disable(DCC_Test_UseCaseArray[i].dccInst);
            }

        if (retVal != SDL_PASS)
        {
            break;
        }

    }

    return retVal;
}

/* Nothing past this point */
