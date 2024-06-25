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
 *  \file     main.c
 *
 *  \brief    This file contains RTI Function test code.
 *
 *  \details  RTI tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "rti_main.h"
#include <dpl_interface.h>
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/sciclient.h>
#endif
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


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
void test_sdl_rti_baremetal_test_app_runner(void);
void test_sdl_rti_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlRtiTest_t  sdlRtiTestList[] = {
    {SDL_RTI_funcTest, "RTI FUNCTION TEST" ,     SDL_APP_TEST_NOT_RUN },
    {NULL,             "TERMINATING CONDITION",  SDL_APP_TEST_NOT_RUN }
};

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
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
#define RTI_NUM_DEVICES 1
uint32_t RTI_devices[RTI_NUM_DEVICES] =
{
    TISCI_DEV_MCU_RTI0,
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
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
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
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
SDL_ESM_config RTI_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xffffffffu, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                },
     /**< esm events enable */

    .priorityBitmap = {0xffffffffu, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                        },
    /**< esm events high priority */
    .errorpinBitmap = {0xffffffffu, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                      },
};
#endif
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
SDL_ESM_config RTI_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                    0x00000000u, 0x00000004u,
                    },
        /**< All events enable: except timer and self test  events, */
        /*    and Main ESM output.Configured based off esmErrorConfig to test high or low priorty events.*/
    .priorityBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                        0x00000000u, 0x00000004u,
                    },
        /**< Configured based off esmErrorConfig to test high or low priorty events. */
    .errorpinBitmap = {0x00000000u, 0x000000e0u, 0x00000000u, 0x00000000u,
                     0x00000000u, 0x00000004u,
                    },
    /**< All events high priority:  */
};
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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

static inline uint32_t RTIDwwdReadTimeOut(uint32_t baseAddr)
{
    uint32_t timeOutValue;

    timeOutValue = ((HW_RD_REG32(baseAddr + RTI_RTIDWDPRLD)
                     << RTI_DWWDPRLD_MULTIPLIER_SHIFT) |
                    RTI_DWD_MIN_PRELOAD_VAL);

    return timeOutValue;
}
/* internal function to read the window size */
static inline uint32_t RTIDwwdReadWinSz(uint32_t baseAddr)
{
    uint32_t   windowSize;
    /* Get configured Window Size */
    windowSize = HW_RD_REG32(baseAddr + RTI_RTIDWWDSIZECTRL);
    return (windowSize);
}

int32_t RTIDwwdIsClosedWindow(uint32_t baseAddr, uint32_t *pIsClosedWindow)
{
    uint32_t closedWindowstatus, currentDownCounter=0, windowSizeShift;
    uint32_t windowStartTime, timeOutValue, windowSize;
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
	uint32_t getBaseAddr;
	SDL_RTI_getBaseaddr(baseAddr,&getBaseAddr);
	baseAddr=getBaseAddr;
#endif
#endif
    int32_t retVal = SDL_EFAIL;
    if ((baseAddr        != ((uint32_t) NULL)) &&
        (pIsClosedWindow != (NULL_PTR)))
    {
        windowSizeShift    = (uint32_t)
                             RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT_SHIFT;
        /* Get configured Window Size */
        windowSize = RTIDwwdReadWinSz(baseAddr);
        switch (windowSize)
        {
            case RTI_DWWD_WINDOWSIZE_100_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_50_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_25_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_12_5_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_6_25_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_3_125_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_3_125_PERCENT_SHIFT;
                break;
            default:
                break;
        }
        timeOutValue    = RTIDwwdReadTimeOut(baseAddr);
        windowStartTime = (timeOutValue /
                           (uint32_t) ((uint32_t) 0x1U << windowSizeShift));
        /* Get current down counter */
        currentDownCounter = HW_RD_REG32(baseAddr + RTI_RTIDWDCNTR);
        if (currentDownCounter > windowStartTime)
        {
            closedWindowstatus = TRUE;
        }
        else
        {
            closedWindowstatus = FALSE;
        }
        /* Update the status */
        *pIsClosedWindow = closedWindowstatus;
        retVal = SDL_PASS;
    }
    return (retVal);
}
#if defined (SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
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
        DebugP_log("Error: Init Failed\r\n");
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

  #if defined (SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX)
    void *ptr = (void *)&arg;
  #endif

    DebugP_log("\n RTI Function Test Application\r\n");

    /* Init dpl */
    sdlApp_dplInit();
	#if !defined (SOC_AM64X) && !defined (SOC_AM243X)
    Sdl_Module_clockEnable();
    Sdl_Module_clockSetFrequency();
	#endif
    /* Initialize MCU RTI module */
    #if defined (SOC_AM263X) || defined (SOC_AM263PX)
    result = SDL_ESM_init(SDL_INSTANCE_ESM0, &RTI_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
    #elif defined (SOC_AM273X)
    result = SDL_ESM_init (SDL_INSTANCE_ESM0,&params,NULL,NULL);
    #elif defined (SOC_AWR294X)
    result = SDL_ESM_init (SDL_INSTANCE_ESM0,&params,NULL,NULL);
    #endif
    #if defined (SOC_AM64X) || defined (SOC_AM243X)
    #if defined (M4F_CORE)
	sdlApp_initRTI();
	result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &RTI_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, ptr);
	#endif
    #endif
    #if defined (SOC_AM64X) || defined (SOC_AM243X)
    #if defined (R5F_CORE)
	sdlApp_initRTI();
	result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &RTI_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
	#endif
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

#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
}


void test_sdl_rti_baremetal_test_app_runner(void)
{
    /* @description:Test runner for RTI tests

       @cores: mcu1_0 */

#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_rti_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
   // print_unityOutputBuffer_usingUARTstdio();
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
