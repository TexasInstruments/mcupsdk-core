/* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     esm_test_main.c
 *
 *  \brief    This file contains ESM test code.
 *
 *  \details  ESM tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "esm_test_main.h"
#include <dpl_interface.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>

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
void test_sdl_esm_baremetal_test_app_runner(void);
void test_sdl_esm_baremetal_test_app (void *args);

void sdlApp_print(char * str)
{
    DebugP_log(str);
}

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlEsmTest_t  sdlEsmTestList[] = {
    {sdl_Esm_posTest, "ESM POSITIVE TEST" ,    SDL_APP_TEST_NOT_RUN },
    {sdl_Esm_negTest, "ESM NEGATIVE TEST",         SDL_APP_TEST_NOT_RUN },
    {NULL,           "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
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

void test_sdl_esm_baremetal_test_app (void *args)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;

    /* Init dpl */
    sdlApp_dplInit();

    sdlApp_print("\n ESM Test Application\r\n");

    for ( i = 0; sdlEsmTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlEsmTestList[i].testFunction();
        sdlEsmTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlEsmTestList[i].testFunction != NULL; i++)
    {
        if (sdlEsmTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \n", sdlEsmTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %d  PASSED \n", i);
        }
    }

}

void test_sdl_esm_baremetal_test_app_runner(void)
{
    UNITY_BEGIN();
    RUN_TEST (test_sdl_esm_baremetal_test_app, 0, NULL);
    UNITY_END();
    return;
}

void test_main(void *args)
{
    test_sdl_esm_baremetal_test_app_runner();

}

/* Nothing past this point */
