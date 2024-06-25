/* Copyright (c) 2022-23 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams hwaTestparamsDSS[2U] =
{
    {
        /* Event BitMap for ECC ESM callback for DSS SEC */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG2_DSS_HWA_GRP2_ERR ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_HWA_ESM_CallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKA */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG2_DSS_HWA_GRP2_ERR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_HWA_ESM_CallbackFunction,
    },
};

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
    int32_t    count;
    uint64_t testStartTime;
    uint64_t testEndTime;
    uint64_t diffTime;
    Drivers_open();
    sdlApp_dplInit();
    DebugP_log("\nHWA TEST START : starting\r\n");
    for(count=0U;count<2U;count++)
    {
        testResult = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &hwaTestparamsDSS[count],NULL,NULL);
        if(testResult == SDL_APP_TEST_PASS )
        {
            break;
        }
        else
        {
            testResult= SDL_APP_TEST_PASS;
        }
    }
    if(testResult == SDL_APP_TEST_PASS )
    {
        DebugP_log("\nESM Initialization for all the SDL HWA Nodes is Done\r\n");
        for ( count = 0; sdlhwaTestList[count].testFunction != NULL; count++)
        {
            sdlhwaTestList[count].testStatus = testResult;
            /* Get start time of test */
            testStartTime = ClockP_getTimeUsec();
            testResult = sdlhwaTestList[count].testFunction();
            /* Record test end time */
            testEndTime = ClockP_getTimeUsec();
            diffTime = testEndTime-testStartTime;
            sdlhwaTestList[count].testStatus = testResult;
            sdlhwaTestList[count].test_time =diffTime;
        }
        for ( count = 0; sdlhwaTestList[count].testFunction != NULL; count++)
        {
            if (sdlhwaTestList[count].testStatus != SDL_APP_TEST_PASS)
            {
                DebugP_log("\nApplications Name: %s  FAILED and Time taken for the Test is %d  micro secs \r\n", sdlhwaTestList[count].name, (uint32_t)sdlhwaTestList[count].test_time);
                testResult = SDL_APP_TEST_PASS;
                break;
            }
            else
            {
                DebugP_log("\nApplications Name: %s  PASSED  and Time taken for the Test is %d  micro secs \r\n", sdlhwaTestList[count].name ,(uint32_t)sdlhwaTestList[count].test_time );
            }
        }

        if (testResult == SDL_APP_TEST_PASS)
        {
            DebugP_log("\nAll tests have passed \r\n");
        }
        else
        {
            DebugP_log("\nFew/all tests Failed \r\n");
        }
    }
    else
    {
        DebugP_log("\nESM Initialization for Few/all the SDL HWA Nodes is Failed\r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
    Drivers_close();
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

int32_t test_main(void)
{
    test_sdl_hwa_baremetal_test_app_runner();

    return 0;
}

/* Nothing past this point */


