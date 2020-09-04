/* Copyright (c) 2022 Texas Instruments Incorporated
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
 *  \file     hwa_main.c
 *
 *  \brief    This file contains hwa function test code.
 *
 *  \details  HWA tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "hwa_main.h"
#include <dpl_interface.h>
#include <kernel/dpl/CacheP.h>

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
/* Unity functions */
void test_sdl_hwa_baremetal_test_app_runner(void);
void test_sdl_hwa_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlHWATest_t  sdlhwaTestList[] = {
    {hwaParityDMA0DMEM0_testExecute,    "HWA_ParityDMA0DMEM0Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM1_testExecute,    "HWA_ParityDMA0DMEM1Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM2_testExecute,    "HWA_ParityDMA0DMEM2Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM3_testExecute,    "HWA_ParityDMA0DMEM3Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM4_testExecute,    "HWA_ParityDMA0DMEM4Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM5_testExecute,    "HWA_ParityDMA0DMEM5Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM6_testExecute,    "HWA_ParityDMA0DMEM6Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0DMEM7_testExecute,    "HWA_ParityDMA0DMEM7Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA0WindowRam_testExecute,"HWA_ParityDMA1WindowRamTest", SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM0_testExecute,    "HWA_ParityDMA1DMEM0Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM1_testExecute,    "HWA_ParityDMA1DMEM1Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM2_testExecute,    "HWA_ParityDMA1DMEM2Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM3_testExecute,    "HWA_ParityDMA1DMEM3Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM4_testExecute,    "HWA_ParityDMA1DMEM4Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM5_testExecute,    "HWA_ParityDMA1DMEM5Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM6_testExecute,    "HWA_ParityDMA1DMEM6Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1DMEM7_testExecute,    "HWA_ParityDMA1DMEM7Test",     SDL_APP_TEST_NOT_RUN },
    {hwaParityDMA1WindowRam_testExecute,"HWA_ParityDMA1WindowRamTest", SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_fsmLockStepExecute,        "HWA_fsmLockStepTest",       SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA0_SEC_test,             "HWA_DMA0SECTest",              SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA0_DED_test,             "HWA_DMA0DEDTest",              SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA0_RED_test,             "HWA_DMA0REDTest",              SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA1_SEC_test,             "HWA_DMA0SECTest",              SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA1_DED_test,             "HWA_DMA0DEDTest",              SDL_APP_TEST_NOT_RUN },
    {SDL_HWA_DMA1_RED_test,             "HWA_DMA0REDTest",              SDL_APP_TEST_NOT_RUN },
    {NULL,                              "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_hwa_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;

    /* Init Dpl */
    sdlApp_dplInit();

    DebugP_log("\n HWA Test Application\r\n");

    for ( i = 0; sdlhwaTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlhwaTestList[i].testFunction();
        sdlhwaTestList[i].testStatus = testResult;
    }
    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlhwaTestList[i].testFunction != NULL; i++)
    {
        if (sdlhwaTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \n", sdlhwaTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \n", sdlhwaTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\n All tests have passed. \n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \n");
    }

#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
}

void test_sdl_hwa_baremetal_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_hwa_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_hwa_baremetal_test_app();
#endif
    return;
}

void test_main(void *args)
{
    test_sdl_hwa_baremetal_test_app_runner();
}

/* Nothing past this point */


