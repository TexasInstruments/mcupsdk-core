/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
 *  \file     lbist_test_main.c
 *
 *  \brief    This file contains LBIST test code for R5 core.
 *
 *  \details  LBIST unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "lbist_test_main.h"
#include <drivers/sciclient.h>
#include <kernel/dpl/DebugP.h>
#include <dpl_interface.h>
#include <sdl/dpl/sdl_dpl.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>

/* #define DEBUG_TEST_CPU_FREQUENCY */

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SCICLIENT_SERVICE_WAIT_FOREVER                    (0xFFFFFFFFU)
/* define the unlock and lock values */
#define KICK0_UNLOCK_VAL 0x68EF3490
#define KICK1_UNLOCK_VAL 0xD172BC5A
#define KICK_LOCK_VAL    0x00000000

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void test_sdl_lbist_test_app_runner(void);
void test_sdl_lbist_test_app(void *args);

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



static int32_t LBIST_appInitBoard(void)
{
    int32_t           testResult = SDL_PASS;
    uint64_t          mcuClkFreq;

    SOC_unlockAllMMR();

    /* Following code is needed to set Osal timing */
    /* Get the clock frequency */
    testResult = Sciclient_pmGetModuleClkFreq(TISCI_DEV_R5FSS0_CORE0_CPU_CLK,
                                              TISCI_DEV_R5FSS0_CORE0_INTERFACE_CLK,
                                              &mcuClkFreq,
                                              SCICLIENT_SERVICE_WAIT_FOREVER);

    return (testResult);
}

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

static int32_t LBIST_appTest(uint32_t testId)
{
    int32_t    testResult;

    switch (testId)
    {
        case LBIST_FUNC_TEST_ID:
            testResult = LBIST_funcTest();
            DebugP_log("\n LBIST Functionality Test");
            if (testResult == SDL_PASS)
            {
                DebugP_log(" Passed.\r\n");
            }
            else
            {
                DebugP_log(" Failed.\r\n");
            }
            break;

        case LBIST_ERROR_TEST_ID:
            testResult = LBIST_errTest();
            DebugP_log("\n LBIST Error Module Test");
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
            DebugP_log("\n [Error] Invalid LBIST test ID.\r\n");
            testResult = SDL_EFAIL;
            break;
    }

    return (testResult);
}

void test_sdl_lbist_test_app(void *args)
{
    uint32_t testId;
    int32_t  testResult;

    testResult = LBIST_appInitBoard();
    if (testResult == SDL_PASS)
    {
        testResult = sdlApp_dplInit();
    }
    else
    {
        DebugP_log("\r\nBoard Init failed. Exiting the app.\r\n");
    }

    DebugP_log("inside test_sdl_lbist_test_app \n");

    for (testId = ((uint32_t)(0U)); testId < LBIST_TOTAL_NUM_TESTS; testId++)
    {
        testResult = LBIST_appTest(testId);
        if (testResult != SDL_PASS)
        {
            break;
        }
    }

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");

    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }
}

void test_sdl_lbist_test_app_runner(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sdl_lbist_test_app,0,NULL);
    UNITY_END();

}

int32_t test_main(void)
{
	Drivers_open();
	Board_driversOpen();
    test_sdl_lbist_test_app_runner();
	Board_driversClose();
	Drivers_close();
    return 0;
}

/* Nothing past this point */
