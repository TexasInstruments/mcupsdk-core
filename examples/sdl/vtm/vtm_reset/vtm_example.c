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
 *  \file vtm_example.c
 *
 *  \brief This file contains functions that provide input event triggers
 *         for the Voltage and Thermal Monitor (VTM) application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <dpl_interface.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <drivers/sciclient.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_vtm.h>
#include <sdl/sdl_esm.h>
#include "vtm_event_trig.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * These are the default offset values which will be used to configure the
 * thresholds for the max temperature out of range alert event. These values
 * may be changed as needed. Note that the values are in milli-degrees celsius.
 */
#define VTM_HI_OFFSET                    (2000)
#define VTM_LO_OFFSET                    (1000)

/*
 * These are the default parameters to control the loop after configuring the
 * thresholds for the max temperature out of range alert event. Increase the 
 * value of VTM_LOOP_DELAY to reduce the frequency of temperature reads.
 * Configure the value of VTM_LOOP_TIMEOUT to configure how long to wait for
 * the required temperature rise. This macro roughly gives the timeout
 * duration in micro seconds. Default value allows for 15 minutes of time. 
 */
#define VTM_LOOP_DELAY                      (1)
#define VTM_LOOP_TIMEOUT    (15 * 60 * 1000000)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t VTM_triggerTh(int32_t hiTemp, int32_t loTemp);
static int32_t VTM_config(void);
static int32_t VTM_setVTMTempThr(int32_t hiTemp, int32_t loTemp);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Completion of Use Case from Input trigger perspective updates these flags */
extern volatile uint32_t    VTM_eventInputTrig[1];
/* Current Use case being run */
extern volatile uint8_t     VTM_currTestCase;

/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * This function modifies the VTM thresholds.
 */
static int32_t VTM_setVTMTempThr(int32_t hiTemp,
                          int32_t loTemp)
{
    int32_t retVal = SDL_PASS;
    SDL_VTM_configTs cfgTs;
    cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT;
    cfgTs.high_temp_in_milli_degree_celsius = hiTemp;
    cfgTs.low_temp_in_milli_degree_celsius = loTemp;

    /* Set the temperature thresholds */
    retVal = SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);

    return retVal;
}

/*
 * This function reads the current temperature in the VTM module
 * and then modifies the VTM thresholds to cause events to trigger.
 * Note that in a typical system, the thresholds would be kept the static, and
 * changes in temperature would cause the events to occur.
 */
static int32_t VTM_triggerTh(int32_t hi_offset,
                            int32_t lo_offset)
{
    int32_t retVal = 0;
    int32_t temp_milli_degrees_read, i;
    int32_t temp_max = 0;
    SDL_VTM_InstTs insTs = SDL_VTM_INSTANCE_TS_0;
    SDL_VTM_adc_code adc_code_read;
    int32_t hiTemp, loTemp;
    SDL_VTM_Stat_val statusVal;
    SDL_VTM_Stat_read_ctrl readCtrl;
    SDL_VTM_configTs cfgTs;
    readCtrl = SDL_VTM_TS_READ_DATA_OUT_VAL;

    /* Enable temp sensor and set it for continuous mode */
    cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;

    /* Mode and Reset status of VTM are to be updated */
    cfgTs.tsCtrl_cfg.valid_map = SDL_VTM_TS_CTRL_MODE_VALID + SDL_VTM_TS_CTRL_RESET_CTRL_VALID;

    /* Value to be set for continuous mode */
    cfgTs.tsCtrl_cfg.mode = SDL_VTM_TS_CTRL_CONTINUOUS_MODE;

    /* Value to be set for sensor operation */
    cfgTs.tsCtrl_cfg.tsReset = SDL_VTM_TS_CTRL_SENSOR_NORM_OP;

    SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);

    /* Get 5 readings of current temperature value, and take the maximum */
    for (i = 0; i < 5; i++) {
        SDL_VTM_getSensorStatus(insTs, &readCtrl, &statusVal);
        adc_code_read = statusVal.data_out;
        (void) SDL_VTM_tsConvADCToTemp (adc_code_read, insTs, \
                                        &temp_milli_degrees_read);
        DebugP_log ("sensor id                       : %d \r\n" \
                    "temp in milli degree celcius    : %d \r\n", \
                    insTs, temp_milli_degrees_read);
        if (temp_milli_degrees_read > temp_max)
        {
            temp_max = temp_milli_degrees_read;
        }
    }

    /*
     * Set hi and lo thresholds relative to the max temperature reading
     * May replace these settings with custom values as per requirement
     * Note that temperature values are in milli-degrees celcius
     */
    hiTemp = temp_max + hi_offset;
    loTemp = temp_max + lo_offset;

    DebugP_log ("vtmTriggerTh: Setting high temp to " \
                 "%d millidegrees Celsius\r\n",
                 hiTemp);
    DebugP_log ("vtmTriggerTh: Setting low temp to " \
                 "%d millidegrees Celsius\r\n",
                 loTemp);

    VTM_setVTMTempThr(hiTemp, loTemp);

    DebugP_log("Finished VTM threshold setting\r\n");

    return (retVal);
}

static int32_t VTM_config(void)
{
    int32_t retVal = 0;
    int32_t temp_milli_degrees_read;
    SDL_VTM_InstTs insTs = SDL_VTM_INSTANCE_TS_0;
    SDL_VTM_adc_code adc_code_read;
    SDL_VTM_Stat_val statusVal;
    SDL_VTM_Stat_read_ctrl readCtrl;
    readCtrl = SDL_VTM_TS_READ_DATA_OUT_VAL;
    uint64_t iterTime, remainingTime, elapsedTime;
    remainingTime = VTM_LOOP_TIMEOUT;

    DebugP_log("VTM_config: starting test with threshold change\r\n");

    /*
     * Set thresholds with offsets VTM_HI_OFFSET and VTM_LO_OFFSET, these values can be changed as per
     * requirement. With default values, SoC will be reset when the temperature rises by 2000 milli-
     * degrees (or 2 degrees) compared to the current temperature. After the reset, SoC will be brought
     * out of reset when temperature falls back to 1000 milli-degrees (1 degree) above current temperature
     */
    retVal = VTM_triggerTh(VTM_HI_OFFSET, VTM_LO_OFFSET);

    DebugP_log("Now waiting for thermal reset...\r\n");

    if (retVal == 0)
    {
        iterTime = ClockP_getTimeUsec();
        /* Wait until SoC reset */
        while(remainingTime > 0)
        {
            SDL_VTM_getSensorStatus(insTs, &readCtrl, &statusVal);
            adc_code_read = statusVal.data_out;
            (void) SDL_VTM_tsConvADCToTemp (adc_code_read, insTs, \
                                            &temp_milli_degrees_read);
            DebugP_log ("temp in milli degree celcius    : %d \r\n", \
                        temp_milli_degrees_read);
            SDL_DPL_delay(VTM_LOOP_DELAY);
            
            /* Calculate time elapsed since last loop iteration */
            elapsedTime = ClockP_getTimeUsec() - iterTime;

            /* Update timestamp of current loop iteration */
            iterTime = ClockP_getTimeUsec();

            /* Update remaining time before timeout */
            if (elapsedTime > remainingTime)
            {
                remainingTime = 0;
            }
            else
            {
                remainingTime -= elapsedTime;
            }           
        }
    }

    /* If the code flow reaches here, the program has timed out or there was an earlier failure */
    if (remainingTime == 0)
    {
        DebugP_log("Application timed out - try reducing VTM_HI_OFFSET or increasing VTM_LOOP_TIMEOUT.\r\n");
        retVal = SDL_EFAIL;
    }
    else
    {
        DebugP_log("Failed to configure VTM Max Temperature Threshold Alert.\r\n");
    }

    return (retVal);
}

/*
* This function deactivates the Max Temperature Out-of-Range Alert
*/
void VTM_deactivateMaxTempOutrngAlert(void)
{
    SDL_VTM_configTs cfgTs;
    cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT_DISABLE;
    SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);
    return;
}

int32_t VTM_runTestCaseTrigger(uint8_t useCaseId)
{
    int32_t       retVal = 0;
    DebugP_log("\r\nStarting Use Case %d \r\n", useCaseId);
    switch(useCaseId)
    {
        case 0:
            /* UC-0: VTM triggered SoC reset due to Max Temperature Out of Range Alert */
            retVal = VTM_config();
            if (retVal == 0)
            {
                DebugP_log("Case 0 - VTM Triggered SoC Reset - Success\r\n");
                VTM_eventInputTrig[useCaseId] = VTM_USE_CASE_STATUS_COMPLETED_SUCCESS;
            }
            else
            {
                DebugP_log("Case 0 - VTM Triggered SoC Reset - Failure\r\n");
                VTM_eventInputTrig[useCaseId] = VTM_USE_CASE_STATUS_COMPLETED_FAILURE;
            }
            break;
        default:
            DebugP_log("ERR: Invalid Test Case ID %d \r\n", useCaseId);
            retVal = -1;
            break;
    }

    return (retVal);
}

/* Nothing past this point */