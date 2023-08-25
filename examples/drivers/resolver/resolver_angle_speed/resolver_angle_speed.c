/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <stdio.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/resolver/v0/resolver.h>

/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */

#define ITERATIONS                      (2000U)
#define MAX_16B_SIGNED_VALUE            (0x7FFF)
#define NUMBER_OF_16B_VALUES            (0x10000)
#define SAMPLING_FREQUENCY_IN_HZ        (20000U)
#define NUMBER_OF_32B_VALUES            (0x100000000)

static HwiP_Object gEpwmHwiObject;


float raw_atan_angle[ITERATIONS] = {0};
float raw_track2_angle[ITERATIONS] = {0};
float raw_track2_velocity[ITERATIONS] = {0};

float sw_track2_angle[ITERATIONS] = {0};
float sw_track2_velocity[ITERATIONS] = {0};
volatile uint32_t gCounter_value[ITERATIONS] ={0};

volatile uint32_t gIsrCount = 0;
volatile bool     gIsrComplete = false;
uint32_t rdcBaseAddr = CONFIG_RESOLVER0_BASE_ADDR;

extern void i2c_io_expander_resolver_adc(void* args);

extern __attribute__((section(".benchmark.code")))
void track2_psuedo(int16_t theta_atan, int16_t* angle_output, int32_t* velocity_output);


void App_Angle2Degrees(int16_t angle, float *angleDegree);
void App_velocity2Rps(int32_t velocity, float *velocityRps);

void App_epwm1ISR(void *args);

void resolver_angle_speed_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /*
    2 EPWM are set up. - EPWM 0 and EPWM 1
    EPWM 0 -
        - generates a syncout upon forced.
    EPWM 1 -
        - is set in up count mode
        - shadow load mode for period, loads on sync in
        - shadow period set for 9999, ie, (9999+1)*5ns or 50000 ns or 20Khz
        - CMPA value is set for 10 (same as soc-delay 20)
        - interrupt is generated for when counter == cmpA

    INT xbar 0 is set for EPWM 1 INT.*/

    DebugP_log("setting up the EPWM ISR!!\r\n");
    int32_t     status;
    HwiP_Params hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwm1ISR;
    /* Marking Highest priority to control loop */
    hwiPrms.priority    = 0;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("setting up the EPWM ISR Complete!!\r\n");

    DebugP_log("Resolver Configured!!\r\n");

    RDC_enableResolver(rdcBaseAddr);
    DebugP_log("Resolver Enabled!!\r\n");

    EPWM_clearEventTriggerInterruptFlag(CONFIG_EPWM1_BASE_ADDR);

    DebugP_log("Forcing the Sync Pulse from the EPMW0!!\r\n");
    EPWM_forceSyncPulse(CONFIG_EPWM0_BASE_ADDR);

    /* the sync pulse forces the EPWM1 timebase counter load with the period value of 9999 and the interrupts will be triggered.*/

    while(gIsrCount < ITERATIONS)
    {
        gIsrComplete = false;
        while(gIsrComplete == false);
    }

    DebugP_log("%d Iterations complete. Printing some of the values\r\n", ITERATIONS);
    DebugP_log("    ANGLES (DEGREES)					||	VELOCITIES (RPS)			\r\n");
    DebugP_log("----------------------------------------------------------------||-----------------------------------------------\r\n");
    DebugP_log("	ATAN		RAW_TRACK2		SW_TRAKC2	||	HW_TRACK2		SW_TRACK2\r\n");

    for(int iter = 0; iter < ITERATIONS; iter+=(ITERATIONS/10))
    {
        DebugP_log("%f\t%f\t%f\t%f\t%f\r\n");
    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}


void App_epwm1ISR(void *args)   // 10000 *5nS recurrance.
{
    /* read the Arc Tan output to gloabl value
    read the track2 angle, velocity to gloabl values
    run the sw track2
    update the global values.
    set a ISR complete variable */

    int16_t temp_atanAngle = 0;
    int16_t temp_swTrack2angle = 0;
    int16_t temp_rawTrack2angle = 0;
    int32_t temp_rawTrack2Velocity = 0;
    int32_t temp_swTrack2Velocity = 0;

    float temp_atanAngleDegree = 0, temp_rawTrack2AngleDegree = 0, temp_swTrack2AngleDegree = 0;
    float temp_rawTrack2VelocityRps = 0, temp_swTrack2VelocityRps = 0;

    /* reading HW results.*/
    temp_atanAngle  = RDC_getArcTanAngle(rdcBaseAddr, RDC_RESOLVER_CORE0);
    temp_rawTrack2angle     = RDC_getTrack2Angle(rdcBaseAddr, RDC_RESOLVER_CORE0);
    temp_rawTrack2Velocity  = RDC_getTrack2Velocity(rdcBaseAddr, RDC_RESOLVER_CORE0);

    /* running swTrack2 loop */
    track2_psuedo(temp_atanAngle, &temp_swTrack2angle, &temp_swTrack2Velocity);

    App_Angle2Degrees(temp_atanAngle, &temp_atanAngleDegree);
    App_Angle2Degrees(temp_swTrack2angle, &temp_rawTrack2AngleDegree);
    App_Angle2Degrees(temp_rawTrack2angle, &temp_swTrack2AngleDegree);

    App_velocity2Rps(temp_rawTrack2Velocity, &temp_rawTrack2VelocityRps);
    App_velocity2Rps(temp_swTrack2Velocity, &temp_swTrack2VelocityRps);

    raw_atan_angle[gIsrCount]       = temp_atanAngleDegree;
    raw_track2_angle[gIsrCount]     = temp_rawTrack2AngleDegree;
    raw_track2_velocity[gIsrCount]  = temp_swTrack2AngleDegree;
    sw_track2_angle[gIsrCount]      = temp_rawTrack2VelocityRps;
    sw_track2_velocity[gIsrCount]   = temp_swTrack2VelocityRps;

    gIsrCount++;
    if(gIsrCount >= ITERATIONS)
    {
        EPWM_disableInterrupt(CONFIG_EPWM1_BASE_ADDR);
    }
    EPWM_clearEventTriggerInterruptFlag(CONFIG_EPWM1_BASE_ADDR);
    gIsrComplete = true;
}

void App_Angle2Degrees(int16_t angle, float *angleDegree)
{
    *angleDegree =  ((float)((angle + (int16_t) MAX_16B_SIGNED_VALUE) * 360))/NUMBER_OF_16B_VALUES;
}
void App_velocity2Rps(int32_t velocity, float *velocityRps)
{
    *velocityRps = ((float)(velocity * SAMPLING_FREQUENCY_IN_HZ))/((float)NUMBER_OF_32B_VALUES);
}
