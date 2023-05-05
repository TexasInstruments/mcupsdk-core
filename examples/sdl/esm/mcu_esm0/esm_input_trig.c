/*
 * ESM Example Application
 *
 * Error signaling module (ESM) Example Application
 *
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
 */
/**
 *  \file esm_input_trig.c
 *
 *  \brief This file contains functions that provide input event triggers
 *         for the Error Signaling Module (ESM) application.
 *
 *  \details  ESM Safety Example module tests
 */

/* For Timer functions */
#include <sdl/dpl/sdl_dpl.h>

#include "esm0_main.h"
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/sdl_esm.h>
#include <sdl/include/sdl_types.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define USE_CASES_RUN         (3)
#define USE_CASES             (3)
#define START_USE_CASE        (0)
#define MAX_ESM_EVENTS_LOGGED (20)
#define APP_ARG (1)

/* #define DEBUG */

static uint32_t totalEventsLogged   = 0;
static uint32_t totalHiEventsLogged = 0;
static uint32_t totalLoEventsLogged = 0;
static uint32_t totalCfgEventsLogged = 0;
int32_t apparg = APP_ARG;
/* ESM event log entry */
typedef struct
{
    SDL_ESM_Inst esmInstance;
    SDL_ESM_IntType      intType;
    uint32_t             grpChannel;
    uint32_t             index;
    uint32_t             intSrc;
    uint8_t              useCaseNum;
} ESM_Example_log_entry_t;
static int32_t deactivateTrigger(SDL_ESM_Inst esmInstType,
                                 SDL_ESM_IntType esmIntType,
                                 uint32_t intEsmSrc);

static ESM_Example_log_entry_t esmEventLog[MAX_ESM_EVENTS_LOGGED];


/* State variable for each test case indicating input event trigger
 * has been completed */
volatile uint32_t gesmEventInputTrig[USE_CASES] = {USE_CASE_STATUS_NOT_RUN,
                                                  USE_CASE_STATUS_NOT_RUN,
                                                  USE_CASE_STATUS_NOT_RUN};

/* State variable for each test case indicating the ISR for the test case
 * has been completed */
volatile uint32_t esmOutputResult[USE_CASES] = {USE_CASE_STATUS_NOT_RUN,
                                                USE_CASE_STATUS_NOT_RUN,
                                                USE_CASE_STATUS_NOT_RUN};

/* State variable for each test case indicating the pin clearing has been
 * completed by pin clearing ISR (applicable only to test cases only
 * with high priority interrupts routed to the pin directly) */
volatile uint32_t gesmPinClearResult[USE_CASES] = {USE_CASE_STATUS_NOT_RUN,
                                                  USE_CASE_STATUS_NOT_RUN,
                                                  USE_CASE_STATUS_NOT_RUN};

volatile uint8_t gcurrTestCase = START_USE_CASE;

static const char *printTestCaseStepResult(uint32_t result);
void ESM_example_printSummary(void);
int32_t ESM_example_init (void);
#if defined (M4F_CORE)
/* Initialization structure for MCU ESM instance */
static SDL_ESM_config ESM_Example_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0xffffffffu, 0x033fffffu, 0x7fffffffu, 0xffffffe7u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, */
    /*    and Main ESM output.Configured based off esmErrorConfig to test high or low priorty events.*/
    .priorityBitmap = {0xffffffffu, 0x013ffff0u, 0x7fffff7fu, 0xffffffe6u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    /**< Configured based off esmErrorConfig to test high or low priorty events. */
    .errorpinBitmap = {0xffffffffu, 0x013ffff0u, 0x7fffff7fu, 0xffffffe6u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
};
#endif

#if defined (R5F_CORE)
static SDL_ESM_config ESM_Example_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000380u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000380u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000380u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#endif
static const char *printEsmIntType(SDL_ESM_IntType esmIntType)
{
    char *pStr;

    switch(esmIntType)
    {
        case SDL_ESM_INT_TYPE_HI:
            pStr = "High Priority ESM event";
            break;
        case SDL_ESM_INT_TYPE_LO:
            pStr = "Low Priority ESM event";
            break;
        case SDL_ESM_INT_TYPE_CFG:
            pStr = "Config ESM event";
            break;
        default:
            pStr = NULL;
            break;
    }

    return pStr;
}

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg)
{
    /* Log the event */
    esmEventLog[totalEventsLogged].useCaseNum  = gcurrTestCase;
    esmEventLog[totalEventsLogged].esmInstance = esmInstType;
    esmEventLog[totalEventsLogged].intType     = esmIntType;
    esmEventLog[totalEventsLogged].grpChannel  = grpChannel;
    esmEventLog[totalEventsLogged].index       = index;
    esmEventLog[totalEventsLogged].intSrc      = intSrc;

    totalEventsLogged++;
    if (esmIntType == SDL_ESM_INT_TYPE_HI) {
        totalHiEventsLogged++;
    } else if (esmIntType == SDL_ESM_INT_TYPE_LO) {
        totalLoEventsLogged++;
    } else {
      totalCfgEventsLogged++;
    }

    /* Any additional customer-specific actions to address ESM event
     * can be added here */


    deactivateTrigger(esmInstType, esmIntType, intSrc);


    /* Print information to screen */
    DebugP_log("\r\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInstType, esmIntType, grpChannel, index, intSrc);
    DebugP_log("  Take action \r\n");

    DebugP_log("  ESM instance #%d, ESM interrupt type = %s\r\n",
                esmInstType, printEsmIntType(esmIntType));

    esmOutputResult[gcurrTestCase]= USE_CASE_STATUS_COMPLETED_SUCCESS;

    return SDL_PASS;
}

/*********************************************************************
* @fn      ESM_example_init
*
* @brief   Initializes Board, Timers, and ESM module
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/
int32_t ESM_example_init (void)
{
    int32_t retValue=0;
    int32_t result;
#if defined (M4F_CORE)

    result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &ESM_Example_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, &apparg);
    if (result != SDL_PASS) {
        /* print error and quit */
        DebugP_log("TIMER_ESM_init: Error initializing MCU ESM: result = %d\r\n", result);

        retValue = -1;
    } else {
        DebugP_log("\r\nTIMER_ESM_init: Init MCU ESM complete \r\n");
    }
#endif
#if defined (R5F_CORE)
	result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ESM_Example_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, &apparg);

	if (result != SDL_PASS) {
        /* print error and quit */
        DebugP_log("TIMER_ESM_init: Error initializing MAIN ESM: result = %d\r\n", result);

        retValue = -1;
    } else {
        DebugP_log("\r\nTIMER_ESM_init: Init MAIN ESM complete \r\n");
    }
#endif
    return retValue;
}


static int32_t deactivateTrigger(SDL_ESM_Inst esmInstType,
                                 SDL_ESM_IntType esmIntType,
                                 uint32_t intEsmSrc)
{
    int32_t retVal = SDL_PASS;
#if defined (M4F_CORE)
    if (esmInstType == SDL_ESM_INST_MCU_ESM0) {
        if (gcurrTestCase == 5) {
            /* UC-6: Configuration interrupt on MCU ESM */
            /* Re-initialize the ESM with the correct values: */
            SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &ESM_Example_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, &apparg);
        }
#endif
#if defined (R5F_CORE)
if (esmInstType == SDL_ESM_INST_MAIN_ESM0) {
        if (gcurrTestCase == 5) {
            /* UC-6: Configuration interrupt on MCU ESM */
            /* Re-initialize the ESM with the correct values: */
            SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ESM_Example_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, &apparg);
        }
#endif
    } else {
        DebugP_log("ERR: Unexpected ESM Instance %d and ESM Interrupt Type %d \r\n",
                    esmInstType, esmIntType);
        retVal = SDL_EFAIL;
    }

   return (retVal);
}

/*********************************************************************
* @fn      ESM_example_printSummary
*
* @brief   Print summary of all the test cases run
*
* @param   None
*
* @return  None
*/
void ESM_example_printSummary(void)
{
    int32_t i;

    DebugP_log("\r\n\r\n");
    DebugP_log("ESM Example Application summary\r\n");
    DebugP_log("-------------------------------\r\n");
    DebugP_log("Completed %d Test Cases\r\n", gcurrTestCase);
    DebugP_log("Received %d High Priority Interrupts\r\n", totalHiEventsLogged);
    DebugP_log("Received %d Low Priority Interrupts\r\n", totalLoEventsLogged);
    DebugP_log("Received %d Config Priority Interrupts\r\n", totalCfgEventsLogged);

    DebugP_log("\r\nTest Case Event Log\r\n");
    DebugP_log("------------------\r\n");
    for (i = 0; i < totalEventsLogged; i++) {
        DebugP_log("\r\nTest Case %d: ESM Call back function called : grpChannel 0x%x, " \
                    "index 0x%x, intSrc 0x%x \r\n",
                    esmEventLog[i].useCaseNum,
                    esmEventLog[i].grpChannel,
                    esmEventLog[i].index,
                    esmEventLog[i].intSrc);
        DebugP_log("  ESM instance #%d, ESM interrupt type = %s\r\n",
                    esmEventLog[i].esmInstance,
                    printEsmIntType(esmEventLog[i].intType));

    }
}

/*
 * This is the main function for the ESM example application.
 */
void esm_example_app(void *args)
{
    int32_t testErrCount = 0;
    int32_t retValue;
    uint8_t i;

    /* Initialize the ESM instances and handlers */
    retValue = ESM_example_init();

    if (retValue < 0) {
        /* print and exit */
        DebugP_log("\r\nERR: ESM_example_init failed");
        testErrCount++;
    }

    DebugP_log("\r\nESM example init complete");

    /* Initialize the Timers for all the Use Cases */
    retValue = esm_timerInit();

    if (retValue < 0) {
        /* print and exit */
        DebugP_log("\r\nERR: Timer initialization failed");
        testErrCount++;
    }
    /* Trigger each use Case */
    for (i = 0; i < USE_CASES; i++) {
        retValue = useCaseTrigger(i);

        if (retValue != 0) {
            DebugP_log("\r\nERR: Use Case Trigger for use Case %d failed \r\n",
                        retValue);
            break;
        }

        while((gesmEventInputTrig[i] == USE_CASE_STATUS_NOT_RUN) ||
              (esmOutputResult[i] == USE_CASE_STATUS_NOT_RUN) ||
              (gesmPinClearResult[i] == USE_CASE_STATUS_NOT_RUN))
        {
#ifdef DEBUG
            DebugP_log("InputTrig = %d, OutputResult = %d, ClearResult = %d\r\n",
                        gesmEventInputTrig[i],
                        esmOutputResult[i],
                        gesmPinClearResult[i]);
#endif
        }
        DebugP_log("\r\nUse Case %d completed: Input Event Trigger = %s, \r\n" \
                    "                       Event Handler Complete = %s, \r\n" \
                    "                       MCU_SAFETY_ERRORn Pin Clear = %s\r\n",
                    i,
                    printTestCaseStepResult(gesmEventInputTrig[i]),
                    printTestCaseStepResult(esmOutputResult[i]),
                    printTestCaseStepResult(gesmPinClearResult[i]));
        gcurrTestCase++;
    }

    /* Check results of all the tests */
    for (i = 0; i < USE_CASES; i++) {
        if ((gesmEventInputTrig[i] != USE_CASE_STATUS_COMPLETED_SUCCESS) ||
            (esmOutputResult[i] != USE_CASE_STATUS_COMPLETED_SUCCESS) ||
            (gesmPinClearResult[i] != USE_CASE_STATUS_COMPLETED_SUCCESS)) {
            testErrCount++;
        }
    }


    /* Print results and logs of the Test Cases */
    ESM_example_printSummary();
    DebugP_log("\r\nESM Example Application: Complete");

    if (testErrCount == 0)
    {
        DebugP_log("\r\n All tests have passed!!\r\r\n");
    }
    else
    {
        DebugP_log("\r\n ESM Example app failed. \r\n");
    }
    return;
}

static const char *printTestCaseStepResult(uint32_t result)
{
    char *pStr;

    switch(result)
    {
        case USE_CASE_STATUS_NOT_RUN:
            pStr = "Step Not yet run";
            break;
        case USE_CASE_STATUS_COMPLETED_SUCCESS:
            pStr = "Step completed successfully";
            break;
        case USE_CASE_STATUS_COMPLETED_FAILURE:
            pStr = "Step completed with failure";
            break;
        default:
            pStr = NULL;
            break;
    }

    return pStr;
}

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

int32_t mcu_esm0_main(void)
{
    sdlApp_dplInit();

    DebugP_log("\r\n ESM Example Application\r\r\n");
	Drivers_open();
	Board_driversOpen();
    (void)esm_example_app(NULL);
	Board_driversClose();
	Drivers_close();
    while (true)
    {
    }
}

/* Nothing past this point */
