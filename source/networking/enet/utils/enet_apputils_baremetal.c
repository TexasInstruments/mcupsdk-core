/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file     enet_apputils_baremetal.c
 *
 * \brief    CPSW batemetal application utility functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <enet.h>
#include "include/enet_apputils.h"
#include "include/enet_apputils_baremetal.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Timer handle for wait function  */
volatile uint32_t gTimeElapsed = 0U;
static TimerP_Handle gDelayTimerHandle = NULL;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetAppUtils_timerISR(uintptr_t arg)
{
    gTimeElapsed = 1U;
}

static void EnetAppUtils_startClock(uint32_t waitTimeMs)
{
    gTimeElapsed = 0;

    TimerP_setPeriodMicroSecs(gDelayTimerHandle,
                              (uint32_t)(waitTimeMs*1000));

    TimerP_start(gDelayTimerHandle);
}

static void EnetAppUtils_stopClock(void)
{
    TimerP_stop(gDelayTimerHandle);

    /* Reset the timer elapsed variable */
    gTimeElapsed = 0U;
}

void EnetAppUtils_timerInit(void)
{
    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);

    timerParams.startMode  = TimerP_StartMode_USER;
    timerParams.runMode    = TimerP_RunMode_ONESHOT;
    timerParams.periodType = TimerP_PeriodType_MICROSECS;
    timerParams.period     = 0U;

    gDelayTimerHandle = TimerP_create(TimerP_ANY,
                                (TimerP_Fxn) &EnetAppUtils_timerISR,
                                &timerParams);
    EnetAppUtils_assert(NULL != gDelayTimerHandle);

    return;
}

void EnetAppUtils_timerDeInit(void)
{
    EnetAppUtils_assert(NULL != gDelayTimerHandle);
    TimerP_delete(gDelayTimerHandle);
}

void EnetAppUtils_wait(uint32_t waitTimeMs)
{
    /* function called without calling EnetAppUtils_timerInit */
    EnetAppUtils_assert(NULL != gDelayTimerHandle);
    EnetAppUtils_startClock(waitTimeMs);
    /* Start timer in one shot mode */
    while (1U != gTimeElapsed);
    EnetAppUtils_stopClock();
}
