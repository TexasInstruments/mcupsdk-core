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
 *  \file     rti_app_main.c
 *
 *  \brief    This file contains RTI Example test code.
 *
 *  \details  RTI tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "rti_app_main.h"

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* None */
/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */
#if defined (SOC_AM263X)
#define SDL_ESM0_INSTANCE SDL_ESM_INST_MAIN_ESM0
#endif
/* R5F Core for AM273x & AWR294x */
#if defined (R5F_INPUTS)
#define SDL_ESM0_INSTANCE SDL_ESM_INST_MSS_ESM
/* C66 Core for AM273x & AWR294x */
#elif defined (C66_INPUTS)
#define SDL_ESM0_INSTANCE SDL_ESM_INST_DSS_ESM
#endif
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
void test_sdl_rti_baremetal_test_app_runner(void);
void test_sdl_rti_baremetal_test_app (void);
/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlRtiTest_t  sdlRtiTestList[] = {
    {SDL_RTI_exampleTest, "RTI EXAMPLE TEST" ,     SDL_APP_TEST_NOT_RUN },
    {NULL,                "TERMINATING CONDITION",  SDL_APP_TEST_NOT_RUN }
};
typedef struct {

    uint32_t moduleId;
    uint32_t clkId;
    uint32_t clkRate;

} SOC_SDL_ModuleClockFrequency;

#if defined (SOC_AM263X)
SOC_SDL_ModuleClockFrequency sdl_gSocModulesClockFrequency[] = {
    { SOC_RcmPeripheralId_WDT0, SOC_RcmPeripheralClockSource_SYS_CLK, 32000 },

    { SOC_MODULES_END, SOC_MODULES_END, SOC_MODULES_END },
};
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
SOC_SDL_ModuleClockFrequency sdl_gSocModulesClockFrequency[] = {
    { SOC_RcmPeripheralId_MSS_WDT, SOC_RcmPeripheralClockSource_SYS_CLK, 200000 },

    { SOC_MODULES_END, SOC_MODULES_END, SOC_MODULES_END },
};
#endif
static int32_t Sdl_Module_clockEnable()
{
    int32_t status;
#if defined (SOC_AM263X)
        status =  SOC_moduleClockEnable(SOC_RcmPeripheralId_WDT0, 1);
#endif
#if defined (SOC_AM273X) || (SOC_AWR294X)
        status =  SOC_moduleClockEnable(SOC_RcmPeripheralId_MSS_WDT, 1);
#endif
        DebugP_assertNoLog(status == SystemP_SUCCESS);

  return status;
}
static int32_t Sdl_Module_clockSetFrequency()
{
    int32_t status;
        status = SOC_moduleSetClockFrequency(
                    sdl_gSocModulesClockFrequency[0].moduleId,
                    sdl_gSocModulesClockFrequency[0].clkId,
                    sdl_gSocModulesClockFrequency[0].clkRate
                    );
        DebugP_assertNoLog(status == SystemP_SUCCESS);
  return status;
}

#if defined (SOC_AM263X)
SDL_ESM_config RTI_Test_esmInitConfig_MAIN =
{
  .esmErrorConfig = {1u, 8u}, /* Self test error config */
  .enableBitmap = {0x00000000u, 0x00000000u, 0x00000001u, 0x00000000u,
              0x00000000u, 0x00000000u, 0x00000000u,0x00000000u},
   /**< All events enable: except clkstop events for unused clocks
    *   and PCIE events */
    /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
  .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000001u, 0x00000000u,
              0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
  /**< All events high priority: except clkstop events for unused clocks
   *   and PCIE events */
  .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000001u, 0x00000000u,
              0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
  /**< All events high priority: except clkstop for unused clocks
   *   and PCIE events */
};
#elif defined (R5F_INPUTS)
SDL_ESM_NotifyParams params =
{
	.groupNumber = 2U,
	.errorNumber = 11U,
	.setIntrPriorityLvl = 1U,
	.enableInfluenceOnErrPin = 1U,
	.callBackFunction = &SDL_ESM_applicationCallbackFunction,
};
#elif defined (C66_INPUTS)
SDL_ESM_NotifyParams params =
{
	.groupNumber = 2U,
	.errorNumber = 0U,
	.setIntrPriorityLvl = 1U,
	.enableInfluenceOnErrPin = 1U,
	.callBackFunction = &SDL_ESM_applicationCallbackFunction,
};
#endif
#if defined (SOC_AM263X)
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
static uint32_t arg;
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
extern int32_t SDL_ESM_applicationCallbackFunction (SDL_ESM_Inst instance, int32_t grpChannel, int32_t vecNum, void *arg);
#endif
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

/*===========================================================================*/
/*                         Local Function definitions                        */
/*===========================================================================*/
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

void test_sdl_rti_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i, result;

    /* Open drivers to open the UART driver for console */
     Drivers_open();
     Board_driversOpen();

#if defined (SOC_AM263X)
    void *ptr = (void *)&arg;
#endif

    DebugP_log("\n RTI Example Test Application\r\n");

    /* Init dpl */
    sdlApp_dplInit();
    Sdl_Module_clockEnable();
    Sdl_Module_clockSetFrequency();

    /* Initialize MCU RTI module */
    #if defined (SOC_AM263X)
    result = SDL_ESM_init(SDL_INSTANCE_ESM0, &RTI_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
    #elif defined (SOC_AM273X)
    result = SDL_ESM_init (SDL_INSTANCE_ESM0,&params,NULL,NULL);
    #elif defined (SOC_AWR294X)
    result = SDL_ESM_init (SDL_INSTANCE_ESM0,&params,NULL,NULL);
    #endif

    if (result != SDL_PASS)
    {
        /* print error and quit */
         DebugP_log("RTI_Test_init: Error initializing MCU ESM: result = %d\r\n", result);
    }
    else
    {
        DebugP_log("\nRTI_Test_init: Init MCU ESM complete \r\n\n");
    }

    for ( i = 0; sdlRtiTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlRtiTestList[i].testFunction();
        sdlRtiTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlRtiTestList[i].testFunction != NULL; i++)
    {
        if (sdlRtiTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \r\n", sdlRtiTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", sdlRtiTestList[i].name);
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
     Board_driversClose();
     Drivers_close();
}


void test_sdl_rti_baremetal_test_app_runner(void)
{
    /* @description:Test runner for RTI tests

       @cores: mcu1_0 */
    test_sdl_rti_baremetal_test_app();
    return;
}

void sdl_rti_example_uc3_main(void *args)
{
    test_sdl_rti_baremetal_test_app_runner();
    /* Stop the test and wait here */
    while (1);
}

/* Nothing past this point */
