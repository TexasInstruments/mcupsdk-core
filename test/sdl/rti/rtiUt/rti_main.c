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
 *  \file     rti_main.c
 *
 *  \brief    This file contains rti test code.
 *
 *  \details  rti tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "rti_main.h"
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/sciclient.h>
#endif

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#define SOC_MODULES_END     (0xFFFFFFFFu)
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
void test_sdl_rti_baremetal_test_app_runner(void);
void test_sdl_rti_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlrtiTest_t  sdlrtiTestList[] = {
    {SDL_RTI_posTest,    "RTI POSITIVE TEST" ,        SDL_APP_TEST_NOT_RUN },
    {SDL_RTI_negTest,    "RTI NEGATIVE TEST" ,        SDL_APP_TEST_NOT_RUN },
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
typedef struct {

    uint32_t moduleId;
    uint32_t clkId;
    uint32_t clkRate;

} SOC_SDL_ModuleClockFrequency;

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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
#if !defined (SOC_AM64X) && !defined (SOC_AM243X)
static int32_t Sdl_Module_clockEnable()
{
    int32_t status;
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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
#endif
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

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#define RTI_NUM_DEVICES SDL_INSTANCE_RTI11_CFG+1
uint32_t RTI_devices[RTI_NUM_DEVICES] =
{
    TISCI_DEV_MCU_RTI0,
    TISCI_DEV_RTI0,
    TISCI_DEV_RTI1,
    TISCI_DEV_RTI8,
    TISCI_DEV_RTI9,
    TISCI_DEV_RTI10,
    TISCI_DEV_RTI11
};

static int32_t sdlApp_initRTI(void)
{
    int32_t status = SDL_PASS;
    uint32_t i;

    for (i = 0; i < RTI_NUM_DEVICES; i++)
    {
       /* Power up RTI */
        status = Sciclient_pmSetModuleState(RTI_devices[i],
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            printf("   RTI Sciclient_pmSetModuleState 0x%x ...FAILED: retValue %d\n",
                        RTI_devices[i], status);
        }
    }

    return status;
}
#endif
/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_rti_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;



    /* Init dpl */
    sdlApp_dplInit();

    DebugP_log("\n rti Test Application\r\n");
	#if !defined (SOC_AM64X) && !defined (SOC_AM243X)
    Sdl_Module_clockEnable();
    Sdl_Module_clockSetFrequency();
	#endif
	#if defined (SOC_AM64X) || defined (SOC_AM243X)
	sdlApp_initRTI();
	#endif
    for ( i = 0; sdlrtiTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlrtiTestList[i].testFunction();
        sdlrtiTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlrtiTestList[i].testFunction != NULL; i++)
    {
        if (sdlrtiTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \r\n", sdlrtiTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", sdlrtiTestList[i].name);
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

void test_sdl_rti_baremetal_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_rti_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_rti_baremetal_test_app();
#endif
    return;
}

void test_main(void *args)
{
	Drivers_open();
	Board_driversOpen();
    test_sdl_rti_baremetal_test_app_runner();
	Board_driversClose();
	Drivers_close();
}

/* Nothing past this point */
