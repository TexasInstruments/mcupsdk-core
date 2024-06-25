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
 *  \file     ecc_bus_safety_main.c
 *
 *  \brief    This file contains ecc bus safety  test code.
 *
 *  \details  dss l3 tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety_main.h"
#include <dpl_interface.h>

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
void test_sdl_ecc_bus_safety_baremetal_test_app_runner(void);
void test_sdl_ecc_bus_safety_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdldssl3Test_t  sdlEccBusSafetyTestList[] = {
    {sdl_ecc_bus_safety_posTest,   "ecc_bus_safety API POSITIVE TEST",    SDL_APP_TEST_NOT_RUN },
    {sdl_ecc_bus_safety_negTest,   "ecc_bus_safety API NEGATIVE TEST",    SDL_APP_TEST_NOT_RUN },
    {NULL,                         "TERMINATING CONDITION",               SDL_APP_TEST_NOT_RUN }
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
        DebugP_log("\nError: Init Failed\r\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_ecc_bus_safety_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;
    Drivers_open();
    /* Init Dpl */
    sdlApp_dplInit();

    DebugP_log("\nECC BUS Safety Test Application\r\n");

    for ( i = 0; sdlEccBusSafetyTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlEccBusSafetyTestList[i].testFunction();
        sdlEccBusSafetyTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlEccBusSafetyTestList[i].testFunction != NULL; i++)
    {
        if (sdlEccBusSafetyTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("\nTest Name: %s  FAILED \r\n", sdlEccBusSafetyTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("\nTest Name: %s  PASSED \r\n", sdlEccBusSafetyTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\nAll tests have passed\r\n");
    }
    else
    {
        DebugP_log("\nFew/all tests Failed \r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
    Drivers_close();
}

void test_sdl_ecc_bus_safety_baremetal_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_ecc_bus_safety_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_ecc_bus_safety_baremetal_test_app();
#endif
    return;
}

void test_main(void *args)
{
    test_sdl_ecc_bus_safety_baremetal_test_app_runner();
}

/* Nothing past this point */
