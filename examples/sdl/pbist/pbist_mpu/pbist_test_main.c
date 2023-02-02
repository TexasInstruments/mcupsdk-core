/*
 *   Copyright (c) Texas Instruments Incorporated 2022-23
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
 *  \file     pbist_test_main.c
 *
 *  \brief    This file contains PBIST test code.
 *
 *  \details  PBIST unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pbist_test_main.h>
#include <kernel/dpl/DebugP.h>
#include <dpl_interface.h>

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

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

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

int32_t PBIST_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

static
int32_t PBIST_appTest(uint32_t testId)
{
    int32_t    testResult;

    switch (testId)
    {
        case PBIST_FUNC_TEST_ID:
            testResult = PBIST_funcTest();
            DebugP_log("\n PBIST Functionality Test");
            if (testResult == SDL_PASS)
            {
                DebugP_log(" Passed.\r\n");
            }
            else
            {
                DebugP_log(" Failed.\r\n");
            }
            break;
        default:
            DebugP_log("\n [Error] Invalid PBIST test ID.\r\n");
            testResult = SDL_EFAIL;
            break;
    }

    return (testResult);
}

void test_sdl_pbist_test_app(void)
{
    uint32_t testId;
    int32_t  testResult;

    for (testId = ((uint32_t)(0U)); testId < PBIST_TOTAL_NUM_TESTS; testId++)
    {
        testResult = PBIST_appTest(testId);
        if (testResult != SDL_PASS)
        {
            break;
        }
    }

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_PASS();
#endif
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_FAIL();
#endif
    }
}

void test_sdl_pbist_test_app_runner(void)
{
#ifdef UNITY_INCLUDE_CONFIG_H
    UNITY_BEGIN();
    RUN_TEST(test_sdl_pbist_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_pbist_test_app();
#endif
}

void test_main(void *args)
{
    /* Declaration of variables */
    int32_t  testResult=SDL_PASS;

    if (testResult == SDL_PASS)
    {
        testResult = PBIST_dplInit();

        if (testResult == SDL_PASS)
        {
            DebugP_log("\nPBIST Test Application\r\n");
            test_sdl_pbist_test_app_runner();
        }
        else
        {
            DebugP_log("\r\nDpl Init failed. Exiting the app.\r\n");
        }
    }
    else
    {
        DebugP_log("\r\nBoard Init failed. Exiting the app.\r\n");
    }


}

/* Nothing past this point */
