/*
 *   Copyright (c) Texas Instruments Incorporated 2019-2022
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
 *  \file     pbist_test_err.c
 *
 *  \brief    This file contains PBIST Error module test code for R5 core.
 *
 *  \details  PBIST Error module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_pbist.h>

#include <pbist_test_cfg.h>

/* DPL API header files */
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t PBIST_errNegativeTest(void)
{
    int32_t                    testResult = SDL_PASS;
    SDL_ErrType_t sdlRet;
    SDL_PBIST_config PBISTConfig;
    SDL_PBIST_configNeg PBISTNegConfig;
    bool PBISTresult;

    /* Call SDL API */
    sdlRet = SDL_PBIST_softReset(NULL);
    if (sdlRet == SDL_PASS)
    {
        DebugP_log("\n  SDL_PBIST_softReset negative test failed \n");
        testResult = -1;
    }

    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_startNeg((SDL_pbistRegs *)PBIST_NEG_TEST_PBIST_CFG_BASE, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_startNeg negative test 1 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_startNeg(NULL, &PBISTNegConfig);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_startNeg negative test 2 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_startNeg(NULL, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_startNeg negative test 3 failed \n");
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_start((SDL_pbistRegs *)PBIST_NEG_TEST_PBIST_CFG_BASE, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_start negative test 1 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_start(NULL, &PBISTConfig);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_start negative test 2 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_start(NULL, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_start negative test 3 failed \n");
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_checkResult((SDL_pbistRegs *)PBIST_NEG_TEST_PBIST_CFG_BASE, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_checkResult negative test 1 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_checkResult(NULL, &PBISTresult);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_checkResult negative test 2 failed \n");
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_checkResult(NULL, NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_checkResult negative test 3 failed \n");
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        sdlRet = SDL_PBIST_releaseTestMode(NULL);
        if (sdlRet == SDL_PASS)
        {
            DebugP_log("\n  SDL_PBIST_releaseTestMode negative test failed \n");
            testResult = -1;
        }
    }

    return (testResult);
}

/* PBIST Error module test */
int32_t PBIST_errTest(void)
{
    int32_t    testResult;

    testResult = PBIST_errNegativeTest();

    return (testResult);
}

/* Nothing past this point */
