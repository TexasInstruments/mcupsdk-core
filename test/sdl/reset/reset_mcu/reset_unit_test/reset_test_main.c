/*
 *  Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file     reset_test_main.c
 *
 *  \brief    This file contains RESET UNIT test code.
 *
 *  \details  RESET tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>

#ifdef UNITY_INCLUDE_CONFIG_H
#include <unity/unity.h>
#include <unity/unity_config.h>
#endif

#include "reset_test_main.h"


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
void test_sdl_reset_baremetal_test_app_runner(void);
void test_sdl_reset_baremetal_test_app (void);
/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlResetTest_t  sdlResetTestList[] = {
    {SDL_Reset_Test,    "RESET  TEST" ,        SDL_APP_TEST_NOT_RUN },
    {NULL,               "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
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


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_reset_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;

    for ( i = 0; sdlResetTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlResetTestList[i].testFunction();
        sdlResetTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlResetTestList[i].testFunction != NULL; i++)
    {
        if (sdlResetTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \r\n", sdlResetTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", sdlResetTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
}

void test_sdl_reset_baremetal_test_app_runner(void)
{
    /* @description:Test runner for RESET tests

       @cores: mcu1_0 */

#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_reset_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_reset_baremetal_test_app();
#endif
    return;
}

int32_t reset_unit_test_main(void)
{
    Drivers_open();
    Board_driversOpen();

    test_sdl_reset_baremetal_test_app_runner();

    Board_driversClose();
    Drivers_close();

	return 0;
}

/* Nothing past this point */