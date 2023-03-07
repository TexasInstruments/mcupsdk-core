/*
 *  Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     dcc_test_main.c
 *
 *  \brief    This file contains DCC Function test code.
 *
 *  \details  DCC tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "dcc_test_main.h"
#include <dpl_interface.h>

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
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
void test_sdl_dcc_baremetal_test_app_runner(void);
void test_sdl_dcc_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlDccTest_t  sdlDccTestList[] = {
    {SDL_DCC_funcTest, "DCC FUNCTION TEST" ,     SDL_APP_TEST_NOT_RUN },
    {NULL,             "TERMINATING CONDITION",  SDL_APP_TEST_NOT_RUN }
};

#if defined (SOC_AM64X) || defined(SOC_AM243X)
#if defined (M4F_CORE)
/* Although the test uses only Main Domain events, MCU domain must be enabled
 * in order to receive the Main domain event notification */
SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config - not used in this test*/
    .enableBitmap = {0x00000007u, 0x00000030u, 0x00000000u,
                },
     /**< Enabling Main domain ESM output and MCU Domain DCC events */
    .priorityBitmap = {0x0000003u, 0x00000030u, 0x00000000u,
                        },
    /**< All events high priority: except low-priority Main ESM output */
    .errorpinBitmap = {0x00000003u, 0x00000030u, 0x00000000,
                      },
    /**< All high priority events to error pin */
};

SDL_ESM_config DCC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config - not used in this test*/
    .enableBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x003f0000u,
                     0x00000000u, 0x00000000u, 0x00000000u,
                    },
     /**< Enabling all DCC events */
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x003f0000u,
                       0x00000000u, 0x00000000u, 0x00000000u,
                      },
    /**< All events high priority */
    .errorpinBitmap = {0x00000000u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu,
                      },
    /**< All high priority events to error pin */
};
#endif
#if defined (R5F_CORE)
SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{
     /**< All high priority events to error pin */
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

SDL_ESM_config DCC_Test_esmInitConfig_MAIN =
{

  /**< All high priority events to error pin */
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,

                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,

                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
 };
 #endif
 #endif

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

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

void test_sdl_dcc_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i, result;

    DebugP_log("\n DCC Function Test Application\r\n");

    /* Init Dpl */
    sdlApp_dplInit();

    /* Initialize DCC module */
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &DCC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, NULL);

    if (result != SDL_PASS)
    {
        /* print error and quit */
        DebugP_log("DCC_Test_init: Error initializing MCU ESM: result = %d\n", result);
    }
    else
    {
        DebugP_log("\nDCC_Test_init: Init MCU ESM complete \n\n");
    }

    /* Initialize MCU DCC module */
    result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &DCC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, NULL);

    if (result != SDL_PASS)
    {
        /* print error and quit */
        DebugP_log("DCC_Test_init: Error initializing MCU ESM: result = %d\n", result);
    }
    else
    {
        DebugP_log("\nDCC_Test_init: Init MCU ESM complete \n\n");
    }

    for ( i = 0; sdlDccTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlDccTestList[i].testFunction();
        sdlDccTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlDccTestList[i].testFunction != NULL; i++)
    {
        if (sdlDccTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \n", sdlDccTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \n", sdlDccTestList[i].name);
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

void test_sdl_dcc_baremetal_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_dcc_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_dcc_baremetal_test_app();
#endif
    return;
}

int32_t test_main(void)
{
	Drivers_open();
	Board_driversOpen();
    test_sdl_dcc_baremetal_test_app_runner();
	Board_driversClose();
	Drivers_close();

    return 0;
}

/* Nothing past this point */