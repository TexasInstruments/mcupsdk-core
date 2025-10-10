/*
 *  Copyright (c) 2025 Texas Instruments Incorporated
 *
 *  VTM Example
 *
 *  Voltage and Thermal Monitor (VTM) Example Application
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
 *  \brief This file contains functions that provide main function
 *         for the Voltage and Thermal Monitor (VTM) reset application.
 *         There is one usecase in this example, which is the VTM
 *         triggered SoC reset example. This usecase enables the Max
 *         temperature out of range alert event and performs SoC reset
 *         when temperature crosses the set threshold.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "vtm_event_trig.h"
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_esm.h>
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <dpl_interface.h>
#include <sdl/include/sdlr.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * There is one usecase in this example, which is the VTM triggered SoC reset
 * example. This usecase enables the Max temperature out of range alert event
 * and performs SoC reset when temperature crosses the set threshold.
 */
#define VTM_USE_CASES          (1)

#define VTM_START_USE_CASE     (0)
#define VTM_THRM_MASK       (0x10)
#define VTM_THRM_SHIFT        (4U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void VTM_deactivateTrigger(void);
static uint32_t VTM_readRstSrc(void);
void VTM_rstExampleRunner(void);
void VTM_rstExample(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * State variable for each test case indicating
 * input event trigger has been completed
 */
volatile uint32_t VTM_eventInputTrig[VTM_USE_CASES] = {VTM_USE_CASE_STATUS_NOT_RUN};

/*
 * State variable for each test case indicating
 * the ISR for the test case has been completed
 */
volatile uint32_t VTM_outputResult[VTM_USE_CASES] = {VTM_USE_CASE_STATUS_NOT_RUN};

/* State variable indicating the current test case */
volatile uint8_t VTM_currTestCase = VTM_START_USE_CASE;

/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void VTM_deactivateTrigger(void)
{
    /* Deactivate Maxtemp Outrng Alert */
    VTM_deactivateMaxTempOutrngAlert();
    /* Mark usecase as success */
    VTM_outputResult[VTM_currTestCase] = VTM_USE_CASE_STATUS_COMPLETED_SUCCESS;
    
    DebugP_log("VTM Maximum Temperature Outrange Alert disabled.\r\n");
    DebugP_log("VTM Reset Example Application completed successfully.\r\n");
    VTM_currTestCase = 1;
}

static uint32_t VTM_readRstSrc(void)
{
    uint32_t regValue = 0;
    uint32_t thrmRst = 0;

    /* Read Reset source register */
    regValue = SOC_getWarmResetCauseMcuDomain();
    
    /* Extract bitfield corresponding to Thermal reset */
    thrmRst = (regValue & VTM_THRM_MASK) >> VTM_THRM_SHIFT;

    return thrmRst;
}

/*
 * This is the main function for the Voltage and Thermal Monitor (VTM) Reset example
 * application. It demonstrates usage of the VTM module to induce an SoC reset when the
 * temperature detected by the VTM sensor is too high and unsafe for device operation.
 */
void VTM_rstExample(void)
{
    int32_t testErrCount = 0;
    int32_t retValue;
    uint8_t i;

    /* Trigger each Test Case */
    for (i = VTM_currTestCase; i < VTM_USE_CASES; i++) 
    {
        retValue = VTM_runTestCaseTrigger(i);
        if (retValue != 0) {
            DebugP_log("\r\nERR: Use Case Trigger for Use Case %d failed \r\n", retValue);
            break;
        }
    }

    /* Check results of all the tests */
    for (i = 0; i < VTM_USE_CASES; i++) 
    {
        if ((VTM_eventInputTrig[i] != VTM_USE_CASE_STATUS_COMPLETED_SUCCESS) ||
            (VTM_outputResult[i] != VTM_USE_CASE_STATUS_COMPLETED_SUCCESS)) 
        {
            testErrCount++;
        }
    }

    /* Print results and logs of the Test Cases */
    DebugP_log("\r\n");
    DebugP_log("VTM Reset Example Application summary\r\n");
    DebugP_log("-------------------------------\r\n");
    DebugP_log("Completed %d Test Case(s)\r\n", VTM_USE_CASES);
    DebugP_log("\r\nVTM Reset Example Application: Complete");
    if (testErrCount == 0)
    {
        DebugP_log("\r\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\n VTM Reset Example app failed. \r\n");
    }
    return;
}

void VTM_rstExampleRunner(void)
{
    /* Read the reset source and check for thermal reset */
    if (VTM_readRstSrc())
    {
        DebugP_log("\r\nReset source was thermal.\r\n");
        VTM_eventInputTrig[VTM_currTestCase] = VTM_USE_CASE_STATUS_COMPLETED_SUCCESS;

        /* Thermal reset detected, hence we can now disable the max temp outrange alert */
        VTM_deactivateTrigger();
    }
    VTM_rstExample();
    return;
}

int32_t test_main(void)
{
    SDL_ErrType_t ret = SDL_PASS;
    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\r\n");
    }
    else
    {
        DebugP_log("\r\nVTM Example Application\r\n");
        (void)VTM_rstExampleRunner();
    }
    return 0;
}

/* Nothing past this point */