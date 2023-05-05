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
 *  \file main.c
 *
 *  \brief This file contains functions that provide input event triggers
 *         for the Error Signaling Module (ESM) application.
 *
 *  \details  ESM Safety Example module tests
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/sdl_esm.h>
#include <sdl/esm/v0/esm.h>
#include "esm0_main.h"
#include <sdl/include/sdl_types.h>
#include <sdl/esm/soc/sdl_esm_soc.h>
#include <drivers/sciclient.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include "ti_dpl_config.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* #define DEBUG */
#define PIN_TIMING_TIMER_ID      (1)
#define HIGH_PRIO_TRIG_TIMER_ID  (2)
#define LOW_PRIO_TRIG_TIMER_ID   (3)

#define PIN_CLEAR_PERIOD_USEC    (10)
#define USE_CASE_STATUS_COMPLETED_SUCCESS (1u)
#define USE_CASE_STATUS_COMPLETED_FAILURE (2u)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t cfgIntrTrigger(uint32_t group);
void timerExpPinDisable(uintptr_t arg);
int32_t SDR_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                const SDL_ESM_ErrorConfig_t *esmErrorConfig);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Completion of Test Case from Input trigger perspective updates these flags */
extern volatile uint32_t    gesmEventInputTrig[7];
/* Completion of Test Case from Output Pin clearing perspective updates these flags */
extern volatile uint32_t    gesmPinClearResult[7];
/* Current test case being run */
extern volatile uint8_t     gcurrTestCase;
uint32_t                    gpStatus;

static uint32_t             gesmPinMinIntervalCycles;
static uint32_t             gesmPinMinIntervalUsec;
static uint32_t             gpinClearTimeCycles;
static bool                 gimmediatePinClear;

static SDL_ESM_Inst gcurrEsmInstance;

/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      esmPinTimeInit
*
* @brief   This checks current timings for the minimum interval time
*          on the MCU_SAFETY_ERRORn pin for eacl Error Signaling Module
*
* @param   pinClearTime: Amount of time in microseconds that the Pin control
*          timer waits before triggering clearing of the MCU_SAFETY_ERRORn pin
*
* @return    0 : Success; < 0 for failures
*/
int32_t esmPinTimeInit(uint32_t pinClearTime)
{
    int32_t retVal = SDL_PASS;
    uint32_t modId;
    uint32_t clkId;
    uint64_t esmInputClk;
    uint32_t esm_base_addr;
    esm_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(SDL_MCU_ESM0_CFG_BASE);
    SDL_ESM_getErrPinLowTimePreload(esm_base_addr, &gesmPinMinIntervalCycles);
#ifdef PRINT_DEBUG
    DebugP_log("  MCU ESM pin minimum interval is %d cycles\r\n",
                gesmPinMinIntervalCycles);
#endif

    if (retVal == SDL_PASS) {
        /* MCU ESM clock */
        modId = TISCI_DEV_MCU_ESM0;
        clkId = TISCI_DEV_MCU_ESM0_CLK;

        retVal = Sciclient_pmGetModuleClkFreq(modId,
                                              clkId,
                                              &esmInputClk,
                                              0xFFFFFFFFU);
        if (retVal == SDL_PASS) {
#ifdef PRINT_DEBUG
            DebugP_log("  MCU ESM input clock is %d\r\n", esmInputClk);
#endif
        }
    }

    if (retVal == SDL_PASS) {
        /* Translate Pin Timer Time (microseconds) into number of ESM cycles */
        gpinClearTimeCycles = (uint32_t)((float)pinClearTime /
                             1000000 * (float)esmInputClk);
        DebugP_log("\r\n  Any clear of MCU_SAFETY_ERRORn pin will first wait " \
                    "%d usecs", pinClearTime);
        /* Translate Minimum Time Interval (cycles) into time (microseconds)*/
        gesmPinMinIntervalUsec = (uint32_t)((float)gesmPinMinIntervalCycles /
                                           (float)esmInputClk * 1000000);
        DebugP_log("\r\n  Minimum Time Interval is %d usecs", gesmPinMinIntervalUsec);
    }
    else {
        retVal = SDL_EFAIL;
    }

    /* If desired, ESMSetErrPinLowTimePreload can be used to change
     * minimum interval time here */

    return retVal;
}

/* TIMER FUNCTIONS */

/*********************************************************************
* @fn      esm_timerInit
*
* @brief   This initializes all timers for the ESM Example Application.
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*/
int32_t esm_timerInit(void)
{
    TimerP_Params   timerParams;

    /* Start Timer and register call back for periodic functions */
    /* Initialize timer parameters */
    TimerP_Params_init(&timerParams);

    timerParams.inputPreScaler = CONFIG_TIMER_ESM_TEST_Pin_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER_ESM_TEST_Pin_INPUT_CLK_HZ;
    timerParams.periodInUsec   = 5000;
    timerParams.oneshotMode    = 1;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER_ESM_TEST_Pin], &timerParams);


    if (esmPinTimeInit(PIN_CLEAR_PERIOD_USEC) != 0) {
       DebugP_log("ERR: Pin Time Init failed\r\n");
       return -1;
    }

    DebugP_log("\r\nESM timer initialization complete\r\n");

    return 0;
}

/*
* This callback is triggered by expiration of the pin disable timer
*/
void timerExpPinDisable(uintptr_t arg)
{
    int32_t retVal = SDL_PASS;
    bool pinStatus;
    SDL_Result sdrStatus;
#if defined (M4F_CORE)
    volatile SDL_ESM_Inst instance = SDL_ESM_INST_MCU_ESM0;
#endif
#if defined (R5F_CORE)
	volatile SDL_ESM_Inst instance = SDL_ESM_INST_MAIN_ESM0;
#endif
    pinStatus = SDL_ESM_getNErrorStatus(instance,&gpStatus);
    if (gpStatus != 0) {
        DebugP_log("\r\n  timerExpPinDisable: Incorrect pin value before clearing\r\n");
        retVal = SDL_EFAIL;
    } else {
    DebugP_log("\r\n  timerExpPinDisable: before clear, ESM instance %d view of " \
                "MCU_SAFETY_ERRORn pin is %d\r\n",
                instance, pinStatus);
    }

    sdrStatus = SDL_ESM_clrNError(instance);

    if (sdrStatus != SDL_PASS) {
        /* If resetNError is called within minimum time interval, then the result of
         * SDR_ESM_resetNError will be failure because pin does not change until
         * minimum time interval has expired */
        if ((gpinClearTimeCycles >= gesmPinMinIntervalCycles) ||
            (gimmediatePinClear)) {
            DebugP_log("\r\n  timerExpPinDisable: ERROR - Failed to clear the error " \
                        "pin from ESM Instance %d\r\n",
                        instance);
            retVal = SDL_EFAIL;
        }
    } else {
#ifdef DEBUG
        DebugP_log("  timerExpPinDisable: Cleared the error pin successfully from " \
                    "ESM Instance %d\r\n",
                    instance);
#endif
    }

    pinStatus = SDL_ESM_getNErrorStatus(instance,&gpStatus);
    if (gpStatus != 1) {
        DebugP_log("\r\n  timerExpPinDisable: Incorrect pin value after clearing\r\n");
        retVal = SDL_EFAIL;
    }
#ifdef DEBUG
    DebugP_log("  timerExpPinDisable: after clear, ESM instance %d view of " \
                "MCU_SAFETY_ERRORn pin is %d\r\n\r\n",
                instance, pinStatus);
#endif

    if (retVal == 0) {
        gesmPinClearResult[gcurrTestCase] = USE_CASE_STATUS_COMPLETED_SUCCESS;
    } else {
        gesmPinClearResult[gcurrTestCase] = USE_CASE_STATUS_COMPLETED_FAILURE;
    }
}

/*
* This function clears timer interrupts for Timer input events to the ESM.
*/
int32_t cfgIntrTrigger(uint32_t group)
{
    int32_t retVal = SDL_PASS;
    uint32_t esm_base_addr;
#if defined (M4F_CORE)
    esm_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(SDL_MCU_ESM0_CFG_BASE);
#endif
#if defined (R5F_CORE)
    esm_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(SDL_ESM0_CFG_BASE);
#endif
    retVal = SDL_ESM_setCfgIntrStatusRAW (esm_base_addr, group);

    return retVal;
}

/* USE CASE FUNCTIONS */

/*
* This function initiates the input trigger event for each use case
*/
int32_t useCaseTrigger(uint8_t useCaseId)
{
    int32_t       retVal = SDL_PASS;
    DebugP_log("\r\nStarting Test Case %d \r\n", useCaseId);
    SDL_ESM_ErrorConfig_t esmErrorConfig;

    switch(useCaseId)
    {
        case 2:
            /* UC-2: Low Priority interrupt on MCU ESM*/
            esmErrorConfig.groupNumber = 1;
            esmErrorConfig.bitNumber = 3;
#if defined (M4F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MCU_ESM0;
#endif
#if defined (R5F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MAIN_ESM0;
#endif
            gesmEventInputTrig[2] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            SDR_ESM_errorInsert (gcurrEsmInstance,&esmErrorConfig);
            /* This use case only has low priority interrupts not routed to
             * the pin, so this flag can be set at beginning of the test */
            gesmPinClearResult[useCaseId] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            break;

        case 0:
            /* UC-0: High Priority interrupt on MCU ESM */
            esmErrorConfig.groupNumber = 2;
            esmErrorConfig.bitNumber = 1;
#if defined (M4F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MCU_ESM0;
#endif
#if defined (R5F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MAIN_ESM0;
#endif
            gesmEventInputTrig[0] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            /* Start Timer to control when external Pin is reset */
            TimerP_start(gTimerBaseAddr[CONFIG_TIMER_ESM_TEST_Pin]);
            SDR_ESM_errorInsert (gcurrEsmInstance,&esmErrorConfig);
            /* gesmEventInputTrig flag is set in expiration of the Hi Event timer */
            /* gesmPinClearResult flag is set in expiration of the Pin timer */
            break;

        case 1:
            /* UC-1: Configuration interrupt on MCU ESM */
#if defined (M4F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MCU_ESM0;
#endif
#if defined (R5F_CORE)
            gcurrEsmInstance = SDL_ESM_INST_MAIN_ESM0;
#endif
            retVal = cfgIntrTrigger(0x1);
            if (retVal == 0){
              gesmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_SUCCESS;
      	    }
      	    else
            {
              gesmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_FAILURE;
      	    }
            /* This test case triggers a configuration interrupt which is not routed to
             * the pin, so this flag can be set at beginning of the test */
            gesmPinClearResult[useCaseId] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            break;

        default:
            DebugP_log("ERR: Invalid Test Case ID %d \r\n", useCaseId);
            retVal = SDL_EFAIL;
            break;
    }

    return (retVal);
}

int32_t SDR_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                const SDL_ESM_ErrorConfig_t *esmErrorConfig)
{
    uint32_t   esmInstBaseAddr;
    int32_t result = SDL_EFAIL;

    if (SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)true)) {
        if (esmErrorConfig != ((void *)0u)) {
          if ((esmErrorConfig->groupNumber < SDL_ESM_MAX_EVENT_MAP_NUM_WORDS)
           && (esmErrorConfig->bitNumber < 32u)) {
                /* Insert error */
                (void)SDL_ESM_setIntrStatusRAW(esmInstBaseAddr,
                                    (esmErrorConfig->groupNumber*32u)
                                    + esmErrorConfig->bitNumber);
                result = SDL_PASS;
            }
        }
    }

    return result;
}
/* Nothing past this point */
