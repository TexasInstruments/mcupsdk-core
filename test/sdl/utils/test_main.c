/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \brief    This file contains sdl_r5_utils.asm test code.
 *
 *  \details  ASM API and  test main
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "test_main.h"

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
static int32_t  sdlApp_initBoard(void);
void  test_sdl_baremetal_app (void);
void  test_sdl_baremetal_app_runner (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlasmTest_t  sdlasmTestList[] = {
    {sdl_apiTest,        "ASM API TEST" ,            SDL_APP_TEST_NOT_RUN },
    {NULL,                  "TERMINATING CONDITION",    SDL_APP_TEST_NOT_RUN }
};

/*===========================================================================*/
/*                         Function definitions                              */
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

void test_sdl_baremetal_app (void)
{
    /* Declarations of variables */
    int32_t         testResult = SDL_APP_TEST_PASS;
    int32_t         i;

#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(BOARD_SOK, status);
#endif

    DebugP_log("\r\n ASM Test Application\r\n");

    for ( i = 0; sdlasmTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlasmTestList[i].testFunction();
        sdlasmTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlasmTestList[i].testFunction != NULL; i++)
    {
        if (sdlasmTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED\r\n", sdlasmTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED\r\n", sdlasmTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\n All tests have passed\r\n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed\r\n");
    }

#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif

    return;
}

void  test_sdl_baremetal_app_runner (void)
{
        /* @description:Test runner for R5 Core tests */

#if defined(UNITY_INCLUDE_CONFIG_H)
        UNITY_BEGIN();
        RUN_TEST (test_sdl_baremetal_app);
        UNITY_END();
        /* Function to print results defined in our unity_config.h file */
        print_unityOutputBuffer_usingUARTstdio();
#else
        test_sdl_baremetal_app();
#endif
        return;
}


int test_main (void)
{

    Drivers_open();
    Board_driversOpen();

    test_sdl_baremetal_app_runner();

    Board_driversClose();
    Drivers_close();

    return (0);

}

