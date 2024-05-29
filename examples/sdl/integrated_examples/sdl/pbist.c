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
 *  \brief    This file contains PBIST example code.
 *
 *  \details  PBIST app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_pbist.h>


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the application interface */


/**
 *  \brief PBIST configuration parameter structure.
 */


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define PBIST_INSTANCE_NAME_MAX_LENGTH    20
#define APP_PBIST_TIMEOUT   (100000000U)

#if defined (R5F0_INPUTS)
#define SDL_INTR_NUM SDL_R5FSS0_CORE0_INTR_PBIST_DONE
#elif defined (R5F1_INPUTS)
#define SDL_INTR_NUM SDL_R5FSS1_CORE0_INTR_PBIST_DONE
#endif

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */

typedef struct PBIST_TestHandle_s
{
    char testName[PBIST_INSTANCE_NAME_MAX_LENGTH];
    SDL_PBIST_inst pbistInst;
    SDL_pbistRegs *pPBISTRegs;
    uint32_t numPBISTRuns;
    SDL_PBIST_configNeg PBISTNegConfigRun;
    uint32_t interruptNumber;
    volatile bool doneFlag;
} PBIST_TestHandle_t;


#if defined R5F0_INPUTS
PBIST_TestHandle_t PBIST_TestHandleArray[1] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_R5FSS0_CORE0_INTR_PBIST_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#elif defined R5F1_INPUTS
PBIST_TestHandle_t PBIST_TestHandleArray[1] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_R5FSS1_CORE0_INTR_PBIST_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#endif

int32_t PBIST_runTest(uint32_t instanceId, bool runNegTest)
{
    int32_t testResult = 0;
    SDL_ErrType_t status;
    bool PBISTResult;
    SDL_PBIST_testType testType;

#if defined (SOC_AM273X) || (SOC_AWR294X)
  if (instanceId == SDL_PBIST_INST_TOP)
  {
    if (runNegTest == true)
    {
        testType = SDL_PBIST_NEG_TEST;
    }
    else
    {
        testType = SDL_PBIST_TEST;
    }
  }
   if (instanceId == SDL_PBIST_INST_DSS)
  {
          if (runNegTest == true)
         {
           testType = SDL_PBIST_NEG_TEST;
         }
         else
         {
             testType = SDL_PBIST_TEST;
         }
      }
#endif

#if defined (SOC_AM263X)
if (runNegTest == true)
{
    testType = SDL_PBIST_NEG_TEST;
}
else
{
    testType = SDL_PBIST_TEST;
}

#endif

    status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, APP_PBIST_TIMEOUT, &PBISTResult);
    if ((status != SDL_PASS) || (PBISTResult == false))
    {
        testResult = SDL_EFAIL;
    }

     return (testResult);
}


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

int32_t pbist_run(void *args)
{
    int32_t    testResult = 0;

    /* Run the test for diagnostics first */
    /* Run test on selected instance */
    testResult = PBIST_runTest(SDL_PBIST_INST_TOP, true);

    if (testResult == 0)
    {
      /* Run test on selected instance */
      testResult = PBIST_runTest(SDL_PBIST_INST_TOP, false);
    }

    return testResult;
}

/* Nothing past this point */
