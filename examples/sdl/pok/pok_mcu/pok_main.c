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
 *  \brief    This file contains POK example code.
 *
 *  \details  POK example main
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "pok_main.h"
#include <kernel/dpl/DebugP.h>
#include <dpl_interface.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_esm.h>
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
/* define the unlock values */
#define KICK0_UNLOCK_VAL 0x68EF3490
#define KICK1_UNLOCK_VAL 0xD172BC5A

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static int32_t  sdlApp_initBoard(void);
void test_sdl_pok_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlPokTest_t  sdlPokTestList[] = {
    {sdlPOK_func,        "POK EXAMPLE UC-1" ,         SDL_APP_TEST_NOT_RUN },
    {sdlPOKInPor_func,   "POR EXAMPLE UC-2" ,         SDL_APP_TEST_NOT_RUN },
    {NULL,               "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
};

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
SDL_ESM_config POK_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000007u, 0x00000000u, 0x0007ffffu,0x00000000u
                },
     /**< All events enable: except clkstop events for unused clocks */
    .priorityBitmap = {0x00000007u, 0x00000000u, 0x0007ffffu,0x00000000u
                        },
    /**< All events high priority: except clkstop events for unused clocks */
    .errorpinBitmap = {0x00000007u, 0x00000000u, 0x0007ffffu,0x00000000u
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and selftest error events */
};
#endif
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
SDL_ESM_config POK_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks */
    .priorityBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks */
    .errorpinBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and selftest error events */
};


SDL_ESM_config POK_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000000u, 0x0007ffffu, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks */
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x0007ffffu, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x0007ffffu, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and selftest error events */
};
#endif
#endif

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
static uint32_t arg;
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\r\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_pok_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;
    int32_t    sdlRet;
    void *ptr = (void *)&arg;
     DebugP_log("\r\n POK Test Application\r\r\n");
    /* Init dpl */
    sdlApp_dplInit();
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MCU,6 );
    /* ESM Setup for POK tests */
    /* Initialize MCU ESM module */
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
    sdlRet = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &POK_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction,ptr);
#endif
#if defined (R5F_CORE)
     sdlRet = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &POK_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction,ptr);
	 sdlRet = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &POK_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction,ptr);
#endif
#endif


   if (sdlRet != SDL_PASS) {
        /* print error and quit */
        DebugP_log("sdlEsmSetupForPOK init: Error initializing MCU ESM: sdlRet = SDL_EFAIL \r\n");
         sdlRet = -1;
    } else {
         DebugP_log("\r\nsdlEsmSetupForPOK init: Init MCU ESM complete \r\n");
    }

    for ( i = 0; sdlPokTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlPokTestList[i].testFunction();
        sdlPokTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlPokTestList[i].testFunction != NULL; i++)
    {
        if (sdlPokTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \r\n", sdlPokTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", sdlPokTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\n Few/all tests Failed \r\n");
    }
}

int32_t test_main(void)
{
    Drivers_open();
	Board_driversOpen();
    test_sdl_pok_baremetal_test_app();
    /* Stop the test and wait here */
	Board_driversClose();
	Drivers_close();
    while (1);
}

/* Nothing past this point */