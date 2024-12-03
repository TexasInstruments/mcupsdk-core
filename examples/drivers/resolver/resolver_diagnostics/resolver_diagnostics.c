/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include "linker_defines.h"

/*
 * Resolver Diagnostics
 * --------------------
 * AM263Px Resolver to Digital Converter offers a wide range of the Diagnostics at 
 * HW level to help improve safety. These are crucial to help support the safety 
 * aspect of the application without actaully consuming the CPU. The following are 
 * the diagnostics that are present in the IP. These are present at each RDC core 
 * within the Resolver IP.
 * 
 * 1. Monitor Sin or Cos offset drift (DOS)
 *     - Performed over the estimated Offset from the DC Offset block of the RDC
 * 2. Monitor Sin or Cos gain drift (DOS)
 *     - Performed over the Gain Estimation from the PG estimation and correction 
 *       block of the RDC
 * 3. Monitor phase drift (phase drift between sin and cos channels) (DOS)
 *     - Performed over the Phase Estimation from the PG estimation and correction 
 *       block of the RDC
 * 4. Monitor excitation frequency degradation or loss (DOS)
 *     - Performed over the input samples from the ADC.
 * 5. Monitor rotational signal integrity (DOS)
 *     - Performed over the demodulated samples after PG correction in RDC
 * 6. Monitor signal integrity by checking Sin2+Cos2=Constant (DOS)
 *     - Performed over the demodulated samples after PG correction in RDC
 * 7. Monitor Sin or Cos saturation or very high amplitude (DOS)
 *     - Performed over the samples collected from the ADC before the DC Offset 
 *       block of the RDC
 * 8. Monitor weak Sin or Cos signal below a threshold (LOS)
 *     - Performed over the Demodulated Samples at PG block of the RDC
 * 9. Monitor track2 loop locking to incoming angle (LOT)
 *     - Performed in the Track2 loop of the RDC.
 * 10. Monitor ADC health through calibration channels (DOS)
 *     - Performed by software trigger on any channel, preferably on 
 *       the Cal Channels with known voltages.
 * 
 * It is important from the Software (Application) to verify / check 
 * the diagnostics periodically by injecting fault. 
 * 
 * Example Description
 * -------------------
 * The example intends to demonstrate a way to use the diagnostic checks. 
 * Also, the example intends to showcase the usage of the static configuration 
 * reads and verification of the static configurations with the initialised values.
 * 
 * Input Configurations
 * --------------------
 * 1. Sequencer Mode 4 is used in the application to avail both RDC for dual motor
 *    (or single motor redundant sensing). Please note that, Sequencer modes 0,1,2 
 *    activate only RDC0 and are used for the single motor sensing, where as, 
 *    Sequencer modes 3,4,5 activate two RDC cores RDC0 and RDC1 for dual motor 
 *    sensing or redundant sensing of single motor.
 * 2. Differential Mode of Sampling is selected.
 * 
 * Excitation Configurations
 * -------------------------
 * 1. Amplitude control is 210 or (84%)
 * 2. Sync in is enabled and corresponding PWMxBar is connected with 
 *    the ISR_SYNC_PWM_EPWM3 Syncout.
 * 3. Over Sampling Ratio is set to be 20.
 * 
 * Resolver Core Configurations
 * ----------------------------
 * Both the Cores RDC0 and RDC1 are configured in exact same manner 
 * for the redundant sensing application.
 * 1. Band Pass Filter is enabled. Auto DC Offset Correction is disabled,
 *    with the manual offset correction values set to be 0.
 * 2. Ideal Sample Mode is set to Mode 3 (manual mode) and Override 
 *    value is 7. Bottom Sampling is not enabled.
 * 3. Phase Gain estimation is enabled, but the Auto Correction is disabled. 
 *    Train limit for the estimation is set to 256 Rotations.
 * 4. Manual Gain correction is set to 2.66 (43697) for both Sine and 
 *    Cosine channels. Manual Phase Correction is set to 0 deg (0).
 * 5. Track2 Configurations include the Kvelfilt Value at 8.
 * 
 * Interrupt Configurations and ISR details
 * ----------------------------------------
 * 1. No interrupts from the RDCs are enabled.
 * 2. ISR_SYNC_PWM_EPWM3 is configured for interrupt, via INTxBar0 
 *    at 100 uS for the Resolver Angle, velocity reads.
 * 3. ISR_SYNC_PWMDIAGNOSTICS_10MS_ISR_EPWM7_EPWM3 is configured for 
 *    interrupt, via INTxBar1 at 10 mS for the Resolver Diagnostics Checks.
 * 
 * SW Track2 details
 * -----------------
 * The example showcases an additional method for computing the angle 
 * and velocity which may be optionally used. For quicker accesses, 
 * the loop needs to be placed at TCM and should be strictly timed to 
 * the excitation signal frequency.# External Connections
 * 
 * Watch variables
 * ---------------
 * 1. testLog is an array of 2 structs, defined for the example purpose. 
 * These contain the test status of the diagnostics performed. 
 * These can be monitored. The following are the values and their meaning.
 *     - TEST_PASS     : 0
 *     - TEST_FAIL     : -1
 *     - TEST_NOT_RUN  : 1
 * 2. raw_atan_angle       :   array, with RDC Core number as index, 
 *                             to read the results from the Atan Angle
 * 3. raw_track2_angle     :   array, with RDC Core number as index, 
 *                             to read the results from the track2 Angle
 * 4. raw_track2_velocity  :   array, with RDC Core number as index, 
 *                             to read the results from the track2 Velocity
 * 5. sw_track2_angle      :   array, with RDC Core number as index, 
 *                             to read the results from the sw_track2 Angle
 * 6. sw_track2_velocity   :   array, with RDC Core number as index, 
 *                             to read the results from the sw_track2 Angle
 * 
 * External Connections
 * --------------------
 * DMRD is used for the connections, visit the example documentation page 
 * for more details
 */

#define SAMPLING_FREQUENCY_IN_HZ        (10000U)
#define MAX_16B_SIGNED_VALUE            (0x7FFFU)
#define NUMBER_OF_16B_VALUES            (0x10000U)
#define NUMBER_OF_32B_VALUES            (0x100000000U)
#define PI_VAL                          (3.1415926535f)

#define VELOCITY_SCALE_FACTOR   ((((float) (SAMPLING_FREQUENCY_IN_HZ)) / ((float) NUMBER_OF_32B_VALUES)))


#define MOTOR0 (0)
#define MOTOR1 (1)

#define MOTOR_KE_VALUE_PER_KRPM ((float)9.28)
#define MOTOR_POLE_PAIRS        (4)
#define MOTOR_MAX_VOLTAGE       (24)  

#define MOTOR_ROTATING (1U)
#define MOTOR_NOT_ROTATING (2U)

uint32_t gMotorState = MOTOR_NOT_ROTATING;
float    gMotorCurrentSpeed_rps = 0;
float    gMotorRefSpeed_rpm = 2000;
float    gMotorSpeedStep_rpm = 10;
uint16_t gMotorPwmPeriodVal  = 5000;
float gVq = 0, gVq_max = 7;

uint32_t gIsrCount = 0;

/* Variables used for Velocity conversion */
volatile uint32_t gSamplingFrequency = SAMPLING_FREQUENCY_IN_HZ;
float   gVelocityScaleFactor = 0;

/* HWI Obj for Interrupt Registeration to read Resovler results */
static HwiP_Object gEpwmHwiObject;

/* HWI Obj for registering a low frequency interrupt to perform safety diagnostics */
static HwiP_Object gDiagnosticsHwiObject;

/* Variables to read angles and velocity from each core */
CONTROL_DATA_SECTION volatile float raw_atan_angle[2]        = {0};
CONTROL_DATA_SECTION volatile float raw_track2_angle[2]      = {0};
CONTROL_DATA_SECTION volatile float raw_track2_velocity[2]   = {0};

/* Variables to read angles and velocity via sw implementation of the track2 loop. */
CONTROL_DATA_SECTION volatile float sw_track2_angle[2]       = {0};
CONTROL_DATA_SECTION volatile float sw_track2_velocity[2]    = {0};

/* variable to store the test logs for the diagnostics */
#define TEST_PASS SystemP_SUCCESS
#define TEST_FAIL SystemP_FAILURE
#define TEST_NOT_RUN ((int32_t) 1)
typedef struct {
    int32_t SinCosOffsetDriftData;
    int32_t ExcFreqDegData;
    int32_t SigIntegritySinSqData;
    int32_t SinCosHighAmpData;
    int32_t SinCosWeakAmpData;
    int32_t SinCosGainDriftData;
    int32_t SinCosPhaseDriftData;
    int32_t RotSigIntegrityData;
}testLog_t;

CONTROL_DATA_SECTION volatile testLog_t testLog[2];

CONTROL_DATA_SECTION volatile Diag_Mon_SinCos_Offset_drift_data          gSinCosOffsetDriftDataInit[2], gSinCosOffsetDriftDataTest[2]; 
CONTROL_DATA_SECTION volatile Diag_Mon_SinCos_Gain_drift_data            gSinCosGainDriftDataInit[2],   gSinCosGainDriftDataTest[2];   
CONTROL_DATA_SECTION volatile Diag_Mon_Cos_Phase_drift_data              gSinCosPhaseDriftDataInit[2],  gSinCosPhaseDriftDataTest[2];  
CONTROL_DATA_SECTION volatile Diag_Mon_ExcFreq_Degradataion_data         gExcFreqDegDataInit[2],        gExcFreqDegDataTest[2];        
CONTROL_DATA_SECTION volatile Diag_Mon_Rotational_Signal_Integrity_data  gRotSigIntegrityDataInit[2],   gRotSigIntegrityDataTest[2];   
CONTROL_DATA_SECTION volatile Diag_Mon_Signal_Integrity_SinSq_CosSq      gSigIntegritySinSqDataInit[2], gSigIntegritySinSqDataTest[2]; 
CONTROL_DATA_SECTION volatile Diag_Mon_Sin_Cos_High_Amplitude            gSinCosHighAmpDataInit[2],     gSinCosHighAmpDataTest[2];     
CONTROL_DATA_SECTION volatile Diag_Mon_Sin_Cos_Weak_Amplitude            gSinCosWeakAmpDataInit[2],     gSinCosWeakAmpDataTest[2];     

uint32_t gRdcBaseAddr = CONFIG_RESOLVER0_BASE_ADDR;

extern void i2c_io_expander_resolver_adc();

extern CONTROL_CODE_SECTION
void track2_psuedo_dual_core(
                   int16_t theta_atan_core0, 
                   int16_t theta_atan_core1, 
                   int16_t* angle_output_core0, 
                   int16_t* angle_output_core1, 
                   int32_t* velocity_output_core0,
                   int32_t* velocity_output_core1
                   );

/* function to convert angle to degrees in range 0-360 deg */
/*inline*/ void App_Angle2Pu(int16_t angle, float *angleDegree);

/* function to convert velocity into RPS */
/*inline*/ void App_velocity2Rps(int32_t velocity, float *velocityRps);

/* ISR configured for 10Khz to read resutls */
CONTROL_CODE_SECTION void App_epwm3_controlISR(void *args);

/* Low frequency ISR for diagnostics check */
CONTROL_CODE_SECTION void App_diagnosticsISR(void *args);

/* Function to setup Diagnostics */
void App_setupDiagnostics(uint32_t base, uint32_t rdcCore);

/* Function to check the static initialized configurations */
CONTROL_CODE_SECTION void App_checkStaticConfigurations(int32_t* status, uint32_t base);

CONTROL_CODE_SECTION int32_t App_checkDiagnostics(uint32_t base, uint32_t rdcCore);

volatile uint64_t gCpuClk_hz;

/* Placeholder for a motor run function */
void App_motorRun(uint32_t motor_instance, float angle_pu, float velocity_rps);

void resolver_diagnostics_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    
    /* setting up resolver related pins on board muxes */
    i2c_io_expander_resolver_adc();
    
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncDisableMask, true);

    DebugP_log("setting up the EPWM ISR...\r\n");
    int32_t     status;
    HwiP_Params hwiPrms, hwiPrms1;

    gCpuClk_hz = SOC_getSelfCpuClk();

    /* 50 uS ISR */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwm3_controlISR;
    hwiPrms.priority    = 0;
    hwiPrms.isPulse     = true;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Setting up the EPWM ISR Complete!!\r\n");

    DebugP_log("Setting up Diagnostics ISR...\r\n");

    /* 10 ms ISR */
    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms1.callback    = &App_diagnosticsISR;
    hwiPrms1.priority    = 2;
    hwiPrms1.isPulse     = true;
    status              = HwiP_construct(&gDiagnosticsHwiObject, &hwiPrms1);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Setting up the Diagnostics ISR Complete!!\r\n");

    DebugP_log("Setting up Diagnostics...\r\n");
    
    App_setupDiagnostics(gRdcBaseAddr, RDC_RESOLVER_CORE0);
    App_setupDiagnostics(gRdcBaseAddr, RDC_RESOLVER_CORE1);
    
    DebugP_log("Diagnostics setup complete!!\r\n");

    RDC_enableResolver(gRdcBaseAddr);
    DebugP_log("Resolver Enabled!!\r\n");


    DebugP_log("Forcing the Sync Pulse from the EPMW0!!\r\n");
    EPWM_forceSyncPulse(ISR_SYNC_PWM_EPWM3_BASE_ADDR);

    EPWM_clearEventTriggerInterruptFlag(ISR_SYNC_PWM_EPWM3_BASE_ADDR);
    EPWM_clearEventTriggerInterruptFlag(DIAGNOSTICS_10MS_ISR_EPWM7_BASE_ADDR);
    
    DebugP_log("Atan angles from each core || Test Status From Each Core\r\n");
    DebugP_log("core0 | core1 || OffsetDrift ExcFreqDeg SigIntegritySinSq HighAmp WeakAmp GainDrift PhaseDrift RotSigIntegrity || OffsetDrift ExcFreqDeg SigIntegritySinSq HighAmp WeakAmp GainDrift PhaseDrift RotSigIntegrity\r\n");
    int counter = 15;
    while(counter--)
    {   
        DebugP_log("%f\t|\t", raw_atan_angle[RDC_RESOLVER_CORE0]*360);
        DebugP_log("%f\t||\t", raw_atan_angle[RDC_RESOLVER_CORE1]*360);
        DebugP_log("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t||\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\r\n",
            testLog[RDC_RESOLVER_CORE0].SinCosOffsetDriftData,
            testLog[RDC_RESOLVER_CORE0].ExcFreqDegData,
            testLog[RDC_RESOLVER_CORE0].SigIntegritySinSqData,
            testLog[RDC_RESOLVER_CORE0].SinCosHighAmpData,
            testLog[RDC_RESOLVER_CORE0].SinCosWeakAmpData,
            testLog[RDC_RESOLVER_CORE0].SinCosGainDriftData,
            testLog[RDC_RESOLVER_CORE0].SinCosPhaseDriftData,
            testLog[RDC_RESOLVER_CORE0].RotSigIntegrityData,
            testLog[RDC_RESOLVER_CORE1].SinCosOffsetDriftData,
            testLog[RDC_RESOLVER_CORE1].ExcFreqDegData,
            testLog[RDC_RESOLVER_CORE1].SigIntegritySinSqData,
            testLog[RDC_RESOLVER_CORE1].SinCosHighAmpData,
            testLog[RDC_RESOLVER_CORE1].SinCosWeakAmpData,
            testLog[RDC_RESOLVER_CORE1].SinCosGainDriftData,
            testLog[RDC_RESOLVER_CORE1].SinCosPhaseDriftData,
            testLog[RDC_RESOLVER_CORE1].RotSigIntegrityData);
        ClockP_sleep(1);
    }

    DebugP_log("All tests have passed!!\r\n");

    EPWM_disableInterrupt(ISR_SYNC_PWM_EPWM3_BASE_ADDR);

    Board_driversClose();
    Drivers_close();
}

CONTROL_CODE_SECTION void App_diagnosticsISR(void *args)
{   
    /* Run Diagnostic Tests */
    int32_t testStatus = SystemP_SUCCESS;

    /* ADC health check using the Calibration channels */
    /* Select the ADC Calibration channel */
    RDC_selectCalibrationChannel(gRdcBaseAddr, RDC_ADC_CAL_CHANNEL_CAL3);
    /* Clear the Calibration status and enable it. */
    RDC_clearCalibrationStatus(gRdcBaseAddr);
    RDC_enableCalibration(gRdcBaseAddr);
    /* Wait on the Calibration complete */
    while(RDC_getCalibrationStatus(gRdcBaseAddr) == false);

    /* Read check against a known value and update the testStatus */
    /* 
    uint16_t adc0_data = RDC_getCalibrationData(gRdcBaseAddr, RDC_CAL_ADC0);
    uint16_t adc1_data = RDC_getCalibrationData(gRdcBaseAddr, RDC_CAL_ADC1);
     */

    /* Check the Static Configurations */
    App_checkStaticConfigurations((int32_t*) &testStatus, gRdcBaseAddr);

    /* Check the Diagnostics */
    if(RDC_isResolverEnabled(gRdcBaseAddr))
    {
        testStatus = App_checkDiagnostics(gRdcBaseAddr, RDC_RESOLVER_CORE0);
        testStatus = App_checkDiagnostics(gRdcBaseAddr, RDC_RESOLVER_CORE1);
    }

    /* Clear the interrupt flag */
    EPWM_clearEventTriggerInterruptFlag(DIAGNOSTICS_10MS_ISR_EPWM7_BASE_ADDR);
}

CONTROL_CODE_SECTION void App_epwm3_controlISR(void *args)   // 10000 *5nS recurrance.
{   
    /* 
        Reads the Angle, Velocity outputs, computes SW Track2.
        Place holder for Motor control code
    */

    volatile int16_t gatanAngle_core0       = 0,    gatanAngle_core1 = 0;
    volatile int16_t gswTrack2Angle_core0   = 0,    gswTrack2Angle_core1 = 0;
    volatile int16_t grawTrack2Angle_core0  = 0,    grawTrack2Angle_core1 = 0;
    volatile int32_t grawTrack2Velocity_core0 = 0,  grawTrack2Velocity_core1 = 0;
    volatile int32_t gswTrack2Velocity_core0  = 0,  gswTrack2Velocity_core1 = 0;

    /* reading HW results.*/
    gatanAngle_core0          = RDC_getArcTanAngle(gRdcBaseAddr, RDC_RESOLVER_CORE0);
    gatanAngle_core1          = RDC_getArcTanAngle(gRdcBaseAddr, RDC_RESOLVER_CORE1);

    grawTrack2Angle_core0     = RDC_getTrack2Angle(gRdcBaseAddr, RDC_RESOLVER_CORE0);
    grawTrack2Angle_core1     = RDC_getTrack2Angle(gRdcBaseAddr, RDC_RESOLVER_CORE1);
    
    grawTrack2Velocity_core0  = RDC_getTrack2Velocity(gRdcBaseAddr, RDC_RESOLVER_CORE0);
    grawTrack2Velocity_core1  = RDC_getTrack2Velocity(gRdcBaseAddr, RDC_RESOLVER_CORE1);

    /* running swTrack2 loop for both cores */
    track2_psuedo_dual_core(
        (int16_t)gatanAngle_core0,
        (int16_t)gatanAngle_core1,
        (int16_t*)&gswTrack2Angle_core0,
        (int16_t*)&gswTrack2Angle_core1,
        (int32_t*)&gswTrack2Velocity_core0,
        (int32_t*)&gswTrack2Velocity_core1
    );

    App_Angle2Pu(gatanAngle_core0,         (float*) (&raw_atan_angle[RDC_RESOLVER_CORE0]));
    App_Angle2Pu(gatanAngle_core1,         (float*) (&raw_atan_angle[RDC_RESOLVER_CORE1]));

    App_Angle2Pu(gswTrack2Angle_core0,     (float*) (&sw_track2_angle[RDC_RESOLVER_CORE0]));
    App_Angle2Pu(gswTrack2Angle_core1,     (float*) (&sw_track2_angle[RDC_RESOLVER_CORE1]));

    App_Angle2Pu(grawTrack2Angle_core0,    (float*) (&raw_track2_angle[RDC_RESOLVER_CORE0]));
    App_Angle2Pu(grawTrack2Angle_core1,    (float*) (&raw_track2_angle[RDC_RESOLVER_CORE1]));
    
    App_velocity2Rps(grawTrack2Velocity_core0,  (float*) (&raw_track2_velocity[RDC_RESOLVER_CORE0]));
    App_velocity2Rps(grawTrack2Velocity_core1,  (float*) (&raw_track2_velocity[RDC_RESOLVER_CORE1]));

    App_velocity2Rps(gswTrack2Velocity_core0,   (float*) (&sw_track2_velocity[RDC_RESOLVER_CORE0]));
    App_velocity2Rps(gswTrack2Velocity_core1,   (float*) (&sw_track2_velocity[RDC_RESOLVER_CORE1]));

    /* Using Track2 Angle and Velocity outputs for the Motor Control */
    App_motorRun(MOTOR0, raw_atan_angle[RDC_RESOLVER_CORE0], raw_track2_velocity[RDC_RESOLVER_CORE0]);
    App_motorRun(MOTOR1, raw_atan_angle[RDC_RESOLVER_CORE1], raw_track2_velocity[RDC_RESOLVER_CORE1]);

    
    /* Clear Interrupt */
    gIsrCount++;
    EPWM_clearEventTriggerInterruptFlag(ISR_SYNC_PWM_EPWM3_BASE_ADDR);
}

inline void App_Angle2Pu(int16_t angle, float *angleDegree)
{   
    /* 
    Converts the angle data into 0-1 range. corresponds to 0-360 deg. 
    User can use their PU code here 
    */
    *angleDegree =  ((float)((angle + 32768.0)))/(32768.0*2);
}
inline void App_velocity2Rps(int32_t velocity, float *velocityRps)
{
    *velocityRps = (float)(velocity) * VELOCITY_SCALE_FACTOR;
}

void App_checkStaticConfigurations(int32_t* status, uint32_t base)
{
    int32_t testStatus = SystemP_SUCCESS;   
    
    RDC_configParams getParams;
    /* initialise the getParams */
    RDC_paramsInit(&getParams);

    /* Read the static configurations */
    RDC_getStaticConfigurations(base, (RDC_configParams*) &getParams);

    /* Verify the static configurations */
    testStatus = RDC_verifyStaticConfigurations(base, (RDC_configParams*) &gRdcParams, (RDC_configParams*) &getParams);

    *status = testStatus;
}


void App_setupDiagnostics(uint32_t base, uint32_t rdcCore)
{
    /* Setting up Diagnostics*/

    /* 
     * Setting up the Offset Drift Diagnostics
     * This Diagnostics are performed before the demodulation of the sine and cosine modulated
     * excitation signals.
     * The Offset Correction Estimation block estimates the possible offset error on the given input.
     * This estimation values can be compared against the thresholds in this diagnostics check.
     *  
     * For the ADC input range of +-2^15, the offset drift threshold can be of 5% in the range
     * The thresholds may correspond to +-3280
     */
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_threshold_hi = (int16_t) 3280;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_threshold_lo = (int16_t)-3280;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_cos_hi   = false;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_cos_lo   = false;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_sin_hi   = false;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_sin_lo   = false;
    gSinCosOffsetDriftDataInit[rdcCore].offset_drift_en   = false;
    RDC_setDiagnosticsSinCosOffsetDriftData(base, rdcCore,
        (Diag_Mon_SinCos_Offset_drift_data *)&(gSinCosOffsetDriftDataInit[rdcCore]));

    /*
     * Setting up the Sine and Cosine Gain Drift Diagnostics.
     * This is performed on the demodulated waveforms of sine and cosine. 
     * the phase gain estimation block, when enabeld, estimates the sine and cosine gains.
     * These estimated gains can be used for the diagnostic checks against thresholds 
     * 
     * For this diagnostics to run meaningfully, 
     *  - The Phase Gain estimation block needs to be enabled.
     *  - Motor Rotation needs to be present. 
     * 
     * The Sine-squared value and cosine-squared value are considered for the diagnostics comparison 
     * against the thresholds. So for a input signal that is at 90% of the range of the ADC input range,
     * the Max gain value will coinside to 0.81 (*2^15) and a minimum can be of 50%.
     * 
     * Hence,
     * the High threshold can be 0.8*2^15 = 26214 and 
     * the Low Threshold can be 0.5*2^15  = 16384.
     * 
     */
    RDC_enablePhaseGainEstimation(base, rdcCore);
    gSinCosGainDriftDataInit [rdcCore].gain_drift_threshold_hi = (uint16_t) 26214;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_threshold_lo = (uint16_t) 16384;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_glitch_count = 5;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_cos_hi = false;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_cos_lo = false;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_sin_hi = false;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_sin_lo = false;
    gSinCosGainDriftDataInit [rdcCore].gain_drift_en     = false;
    RDC_setDiagnosticsSinCosGainDriftData(base, rdcCore, 
        (Diag_Mon_SinCos_Gain_drift_data *)&(gSinCosGainDriftDataInit[rdcCore]));

    /* 
     * Setting up the Cosine Phase Drift Diagnostics.
     * This Diagnostics will require Rotation.
     * This is performed on the demodulated waveforms of sine and cosine. 
     * The Phase Gain Estimation block, when enabled, esimates the cosine phase offset.
     * the Phase estimation final value, phaseestimatefinal = phase_error*(pi/180)*(2^15) degrees.
     * 
     * Typical threshold could be of 3 degrees or that correspond to 3*(pi/180)*(2^15) or 1716.
     * Hence,
     * High threshold can be of 1716
     * Low threshold can be of -1716 
     *
     * 
     */
    // RDC_enablePhaseGainEstimation(base, rdcCore); // Since enabled earlier
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_threshold_hi = (int16_t)1716;
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_threshold_lo = (int16_t)-1716;
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_glitch_count = 0;
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_cos_hi  = false;
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_cos_lo  = false;
    gSinCosPhaseDriftDataInit [rdcCore].phase_drift_en      = false;
    RDC_setDiagnosticsCosPhaseDriftData(base, rdcCore, 
        (Diag_Mon_Cos_Phase_drift_data *)&(gSinCosPhaseDriftDataInit[rdcCore]));

    /*      
     * Setting up the Excitation Frequency Degradation Diagnostics 
     * 
     * This is performed on the non-demodulated waveforms. the rotation is not necessary for this 
     * diagnostics check. For a given angle, the exciation signal will be present on either sine or
     * cosine input. (note this is not applicable for inductive resolver sensors, which do not have the
     * exciatation signal in the sine and cosine outputs).
     * 
     * There is a counter that rus at fexcitation * 1000, this should be around 500 count for each 
     * zero crossing of the excitation singal on the modulated sine or cosine input.
     * 
     * Hence, 
     * the High threshold can be 500 + 10 counts 
     * the low threshold can be 500 - 10 counts
     * 
     * The excitation level is essentially, an accepted count value when
     * the input value crossing from +exctation level to -exciation level. this needs to be higher than 
     * noise level but smaller. a good value could be 255 (compared +2^15 positive peak)
     * 
     * Glitch count :
     * - minimum number of times this error scenario to occur before flagging an error flag.
     */
    gExcFreqDegDataInit [rdcCore].excfreqdetected_sin   = 0;    // read only initialising to zero
    gExcFreqDegDataInit [rdcCore].excfreqdetected_cos   = 0;    // read only initialising to zero
    gExcFreqDegDataInit [rdcCore].excfreqdrift_threshold_hi = (uint16_t)550;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_threshold_lo = (uint16_t)450;
    gExcFreqDegDataInit [rdcCore].excfreq_level = (uint16_t)255;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_glitchcount  = 5;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_hi       = false;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_cos_lo   = false;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_sin_lo   = false;
    gExcFreqDegDataInit [rdcCore].excfreqdrift_en       = false;
    RDC_setDiagnosticsExcFreqDegradationData(base, rdcCore, 
        (Diag_Mon_ExcFreq_Degradataion_data *) &(gExcFreqDegDataInit[rdcCore]));

    /* 
     * Setting up the Rotational Signal Integrity Check
     * This is performed on the demodulated sine and cosine signals and after Phase Gain correction.
     * Rotation is required for meaning full check of this diagnostics.
     * 
     * Ideally, there is only 1 zero crossing on Sine between 2 zero crossings of Cosine sinal, the vice
     * versa is true too. Also, when one of them is at zero crossing, the other will be either at a positive 
     * or a negative peak.
     * 
     * rotfreq_level : the value on sine or cosine changing from +rotfreq_level to -rotfreq_level 
     * is considered zero crossing. Good Value could be 255
     * 
     * rotpeak_level : the value over the +rotpeak_level is considered positive peak and 
     * below -rotpeak_level is considered negative peak. 
     * If the signal is around 80% of ADC input range, the Gain is 0.7, then good value could be 0.7*0.8*2^15
     * = 18350
     */
    gRotSigIntegrityDataInit [rdcCore].cos_neg_zc_peak_mismatch_err = false ;
    gRotSigIntegrityDataInit [rdcCore].cos_pos_zc_peak_mismatch_err = false ;
    gRotSigIntegrityDataInit [rdcCore].sin_neg_zc_peak_mismatch_err = false ;
    gRotSigIntegrityDataInit [rdcCore].sin_pos_zc_peak_mismatch_err = false ;
    gRotSigIntegrityDataInit [rdcCore].sin_multi_zc_error_err   = false ;
    gRotSigIntegrityDataInit [rdcCore].cos_multi_zc_error_err   = false ;
    gRotSigIntegrityDataInit [rdcCore].cos_multi_zc_error_count = 0;    // read only initialising to zero
    gRotSigIntegrityDataInit [rdcCore].sin_multi_zc_error_count = 0;    // read only initialising to zero
    gRotSigIntegrityDataInit [rdcCore].rotpeak_level    = (uint16_t)18350;
    gRotSigIntegrityDataInit [rdcCore].rotfreq_level    = (uint16_t)255;
    gRotSigIntegrityDataInit [rdcCore].zero_cross_rot_en    = false;
    RDC_setDiagnosticsRotationalSignalIntegrityData(base, rdcCore, (Diag_Mon_Rotational_Signal_Integrity_data *) &(gRotSigIntegrityDataInit[rdcCore]));

    /* 
     * Setting up the Motor Signal Integrity by Checking "Sin^2 + Cos^2 = Constant" Diagnostics
     * 
     * Rotation is not necessary for this Check.
     * The Diagnostics are perform on the Demodulated Sine and Cosine Signals before the Phase Gain Correction.
     * 
     * The Max values of Sine or Cosine samples are +-2^15. the squared values max out at 2^30. The lower
     * 16 Bits are ignored. ie., the Max value is 2^14. 
     * 
     * Considering the input signals are 90% of the ADC input range, the max value of the sum of squares of sine
     * and cosine is, 0.81*2^14 or ~13271. if the minimum acceptable is around 70% of the ADC input range
     * then the Squares sum be 0.49*2^14 or ~8028.
     * 
     * Glitch Count : Number of squared sums that are above or below thresholds to flag the error.  
     */
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_threshold_hi = (uint16_t)13271;
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_threshold_lo = (uint16_t)8028;
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_glitchcount = 5;
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_cossq = 0;          // read only initialising to zero
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_sinsq = 0;          // read only initialising to zero
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_hi = false;
    gSigIntegritySinSqDataInit [rdcCore].sinsqcossq_lo = false;
    RDC_setDiagnosticsSignalIntegritySquareSumData(base, rdcCore, (Diag_Mon_Signal_Integrity_SinSq_CosSq*)&(gSigIntegritySinSqDataInit[rdcCore]));

    /*
     * Setting up the Sine or Cosine High Amplitude Diagnostics
     * 
     * Rotation is not necessary for this check. this diagnostics is performed over the Sine and Cosine samples
     * that are before Offset Correction and Demodulation.
     * 
     * Typically, the ADC inputs should not be over the ADC input range (0V to 3.2V in Single Ended and 
     * -1.6V to 1.6 V in Differential mode) and if are over the range, the samples saturate and 
     * the offset correction cannot reproduce the actual sine and cosine waveforms.
     * 
     * Hence this Diagnostics is performed on the samples of Sine or Cosine (even if one is over threshold, this flags error)
     * 
     * Considering 90% of ADC input range, the max value could be 0.9*2^15 or ~290490.
     * 
     * Glitch count : Number of times Sine or Cosine sample is over this threshold to flag error. 
     * 
     */
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_threshold    = (uint16_t) 29490;
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_glitchcount  = 5;
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_sin_value    = 0;    // read only initialising to zero
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_cos_value    = 0;    // read only initialising to zero
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_sin_error    = false;
    gSinCosHighAmpDataInit [rdcCore].highAmplitude_cos_error    = false;
    RDC_setDiagnosticsHighAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_High_Amplitude*)&(gSinCosHighAmpDataInit[rdcCore]));
    
    /* 
     * Setting up the Sine or Cosine Low Amplitude Diagnostics
     * 
     * Rotation is not necessary for this check. this diagnostics is performed over the Sine and Cosine samples
     * after Offset Correction and Demodulation but before Phase Gain Correction.
     * 
     * Typically, the ADC inputs should not be below the ADC input range (0V to 3.2V in Single Ended and 
     * -1.6V to 1.6 V in Differential mode) and if are below the range, the samples saturate and 
     * the offset correction cannot reproduce the actual sine and cosine waveforms.
     * 
     * Hence this Diagnostics is performed on the demodulated samples of Sine or Cosine. Not both should lower 
     * than this low threshold.
     * 
     * Considering 90% of ADC input range, the min value could be between 2^13 and 2^12. Lets consider 2^13
     * 
     * Glitch count : Number of times Sine and Cosine samples both are below this threshold to flag error. 
     * 
     */
    gSinCosWeakAmpDataInit [rdcCore].lowAmplitude_threshold     = (uint16_t)8192;
    gSinCosWeakAmpDataInit [rdcCore].lowAmplitude_glitchcount   = 5U;
    gSinCosWeakAmpDataInit [rdcCore].lowAmplitude_error         = false;
    gSinCosWeakAmpDataInit [rdcCore].lowAmplitude_sin_value     = 0U;    // read only initialising to zero
    gSinCosWeakAmpDataInit [rdcCore].lowAmplitude_cos_value     = 0U;    // read only initialising to zero
    RDC_setDiagnosticsWeakAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_Weak_Amplitude*)&(gSinCosWeakAmpDataInit[rdcCore]));

    RDC_clearCoreInterrupt(base, rdcCore, RDC_INTERRUPT_SOURCE_ALL);

}

int32_t App_checkDiagnostics(uint32_t base, uint32_t rdcCore)
{   
    int32_t testStatus = SystemP_SUCCESS;

    testLog[rdcCore].SinCosOffsetDriftData = TEST_NOT_RUN;
    testLog[rdcCore].ExcFreqDegData = TEST_NOT_RUN;
    testLog[rdcCore].SigIntegritySinSqData = TEST_NOT_RUN;
    testLog[rdcCore].SinCosHighAmpData = TEST_NOT_RUN;
    testLog[rdcCore].SinCosWeakAmpData = TEST_NOT_RUN;
    testLog[rdcCore].SinCosGainDriftData = TEST_NOT_RUN;
    testLog[rdcCore].SinCosPhaseDriftData = TEST_NOT_RUN;
    testLog[rdcCore].RotSigIntegrityData = TEST_NOT_RUN;

    /* Setting up Diagnostics*/

    /* 
     * Checking the Sine and Cosine Offset Drift Diagnostics
     * In order to Guarantee an error, configuring the offsets to a very small value
     */
    RDC_getDiagnosticsSinCosOffsetDriftData(base, rdcCore, (Diag_Mon_SinCos_Offset_drift_data *)&(gSinCosOffsetDriftDataTest[rdcCore]));
    gSinCosOffsetDriftDataTest[rdcCore].offset_drift_threshold_hi =  5;
    gSinCosOffsetDriftDataTest[rdcCore].offset_drift_threshold_lo = -5;
    gSinCosOffsetDriftDataTest[rdcCore].offset_drift_en   = false;

    RDC_setDiagnosticsSinCosOffsetDriftData(base, rdcCore, (Diag_Mon_SinCos_Offset_drift_data *)&(gSinCosOffsetDriftDataTest[rdcCore]));
    ClockP_usleep(1);
    RDC_getDiagnosticsSinCosOffsetDriftData(base, rdcCore, (Diag_Mon_SinCos_Offset_drift_data *)&(gSinCosOffsetDriftDataTest[rdcCore]));
    if(
        (gSinCosOffsetDriftDataTest[rdcCore].offset_drift_cos_hi   == false) || 
        (gSinCosOffsetDriftDataTest[rdcCore].offset_drift_cos_lo   == false) || 
        (gSinCosOffsetDriftDataTest[rdcCore].offset_drift_sin_hi   == false) || 
        (gSinCosOffsetDriftDataTest[rdcCore].offset_drift_sin_lo   == false))
    {
        testLog[rdcCore].SinCosOffsetDriftData = TEST_FAIL;
    }else
    {
        /* Reset Diagnostics */
        RDC_setDiagnosticsSinCosOffsetDriftData(base, rdcCore, (Diag_Mon_SinCos_Offset_drift_data *)&(gSinCosOffsetDriftDataInit[rdcCore]));

        /* Clear Flags */
        uint32_t interruptSource = RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR |\
                                   RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR |\
                                   RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR |\
                                   RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR;
        RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
        testLog[rdcCore].SinCosOffsetDriftData = TEST_PASS;
    }

    /* 
     * Checking the Excitation Frequency Degradation Diagnostics 
     * 
     * In order to trigger a fault, the thresholds can be a small pair or a high pair.d
     * 
     * Hence, 
     * 1. the High threshold can be 50 and the low threshold can be 40
     * 2. the High threshold can be 1000 and the low threshold can be 9500
     * 
     * Glitch count :
     * - minimum number of times this error scenario to occur before flagging an error flag.
     */
    RDC_getDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));
    gExcFreqDegDataTest [rdcCore].excfreqdrift_threshold_hi = 50;
    gExcFreqDegDataTest [rdcCore].excfreqdrift_threshold_lo = 40;
    gExcFreqDegDataTest [rdcCore].excfreqdrift_en  = false;

    RDC_setDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));

    /* The wait is for the counter to run upto 1 excitation frequency time period */
    ClockP_usleep((uint32_t) ((float) (1000000.0)/gSamplingFrequency));

    RDC_getDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));
    if(
        gExcFreqDegDataTest [rdcCore].excfreqdrift_hi       == false
    )
    {
        testLog[rdcCore].ExcFreqDegData = TEST_FAIL;
    }else
    {
        RDC_getDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));
        gExcFreqDegDataTest [rdcCore].excfreqdrift_threshold_hi = 1000;
        gExcFreqDegDataTest [rdcCore].excfreqdrift_threshold_lo = 9500;
        RDC_setDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));
        
        /* The wait is for the counter to run upto 1 excitation frequency time period */
        ClockP_usleep((uint32_t) ((float) (1000000.0)/gSamplingFrequency));
        
        RDC_getDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataTest[rdcCore]));
        
        if(
            (gExcFreqDegDataTest [rdcCore].excfreqdrift_cos_lo   == false) &&
            (gExcFreqDegDataTest [rdcCore].excfreqdrift_sin_lo   == false))
        {
            testLog[rdcCore].ExcFreqDegData = TEST_FAIL;
        }else
        {
            /* Reset Diagnostics */
            RDC_setDiagnosticsExcFreqDegradationData(base, rdcCore, (Diag_Mon_ExcFreq_Degradataion_data *)&(gExcFreqDegDataInit[rdcCore]));

            /* Clear Flags */
            uint32_t interruptSource = RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_LO_ERR |\
                                    RDC_INTERRUPT_SOURCE_OFFSETDRIFT_SIN_HI_ERR |\
                                    RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_LO_ERR |\
                                    RDC_INTERRUPT_SOURCE_OFFSETDRIFT_COS_HI_ERR;
            RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
            testLog[rdcCore].ExcFreqDegData = TEST_PASS;
        }
    }
    
    /* 
     * Checking the Motor Signal Integrity by Checking "Sin^2 + Cos^2 = Constant" Diagnostics
     * 
     * To trigger a fault, lets configure high threshold to a lower value and check flag status
     * Also, low threshold to a higher value and check flag status.
     *   
     */

    RDC_getDiagnosticsSignalIntegritySquareSumData(base, rdcCore, (Diag_Mon_Signal_Integrity_SinSq_CosSq *)&(gSigIntegritySinSqDataTest[rdcCore]));

    gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_threshold_hi = 100;
    gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_threshold_lo = 16000;
    gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_hi = false;
    gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_lo = false;

    RDC_setDiagnosticsSignalIntegritySquareSumData(base, rdcCore, (Diag_Mon_Signal_Integrity_SinSq_CosSq *)&(gSigIntegritySinSqDataTest[rdcCore]));
    
    /* Considering the threshold hits everytime, there should be glitchcount number of samplings done to flag */
    ClockP_usleep((uint32_t)((float)(1000000.0 * ((float)gSigIntegritySinSqDataTest[rdcCore].sinsqcossq_glitchcount))/SAMPLING_FREQUENCY_IN_HZ));

    RDC_getDiagnosticsSignalIntegritySquareSumData(base, rdcCore, (Diag_Mon_Signal_Integrity_SinSq_CosSq *)&(gSigIntegritySinSqDataTest[rdcCore]));
    if((gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_hi == false) || 
       (gSigIntegritySinSqDataTest [rdcCore].sinsqcossq_lo == false))
    {
        testLog[rdcCore].SigIntegritySinSqData = TEST_FAIL;
    }else
    {
        /* Reset Diagnostics */
        RDC_setDiagnosticsSignalIntegritySquareSumData(base, rdcCore, (Diag_Mon_Signal_Integrity_SinSq_CosSq *)&(gSigIntegritySinSqDataInit[rdcCore]));

        /* Clear Flags */
        uint32_t interruptSource = RDC_INTERRUPT_SOURCE_SINSQCOSSQ_HI_ERR | RDC_INTERRUPT_SOURCE_SINSQCOSSQ_LO_ERR;
                                
        RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
            testLog[rdcCore].SigIntegritySinSqData = TEST_PASS;
    }

    /*
     * Checking the Sine or Cosine High Amplitude Diagnostics
     * 
     * To Trigger a fault configure High Threshold to low value
     */
    RDC_getDiagnosticsHighAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_High_Amplitude*)&(gSinCosHighAmpDataTest[rdcCore]));
    gSinCosHighAmpDataTest [rdcCore].highAmplitude_threshold    = 40;
    gSinCosHighAmpDataTest [rdcCore].highAmplitude_sin_error    = false;
    gSinCosHighAmpDataTest [rdcCore].highAmplitude_cos_error    = false;

    RDC_setDiagnosticsHighAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_High_Amplitude*)&(gSinCosHighAmpDataTest[rdcCore]));
    
    /* There needs to be glitchcount amount of samples (considering all the samples are hitting faults)*/
    ClockP_usleep((uint32_t) ((float) (gSinCosHighAmpDataTest[rdcCore].highAmplitude_glitchcount)*1.0/gSamplingFrequency));
    
    RDC_getDiagnosticsHighAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_High_Amplitude*)&(gSinCosHighAmpDataTest[rdcCore]));
    if(
        (gSinCosHighAmpDataTest [rdcCore].highAmplitude_sin_error == false) &&
        (gSinCosHighAmpDataTest [rdcCore].highAmplitude_cos_error == false) )
    {
        testLog[rdcCore].SinCosHighAmpData = TEST_FAIL;
        // return testStatus;
    }else
    {
        /* Reset Diagnostics */
        RDC_setDiagnosticsHighAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_High_Amplitude *)&(gSinCosHighAmpDataInit[rdcCore]));

        /* Clear Flags */
        uint32_t interruptSource = RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_COS_FAULT_ERR | RDC_INTERRUPT_SOURCE_HIGHAMPLITUDE_SIN_FAULT_ERR;
                                
        RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
        testLog[rdcCore].SinCosHighAmpData = TEST_PASS;
    }

    /* 
     * Checking the Sine or Cosine Low Amplitude Diagnostics
     * 
     * To trigger a fault, Configuring low threshold to a higher value like 2^14
     */
    RDC_getDiagnosticsWeakAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_Weak_Amplitude*)&(gSinCosWeakAmpDataTest[rdcCore]));
    gSinCosWeakAmpDataTest [rdcCore].lowAmplitude_threshold     = 16384;
    gSinCosWeakAmpDataTest [rdcCore].lowAmplitude_error         = false;
    RDC_setDiagnosticsWeakAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_Weak_Amplitude*)&(gSinCosWeakAmpDataTest[rdcCore]));
    
    /* 
     * After demodulation, but before Phase Gain estimation/ correction logic 
     * So considering the thresholds are too low that all the samples hit the threshold,
     * there should be wait up until the glitch count
     */
    ClockP_usleep((uint32_t) ((float) (gSinCosWeakAmpDataTest[rdcCore].lowAmplitude_glitchcount)*1.0/gSamplingFrequency));

    RDC_getDiagnosticsWeakAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_Weak_Amplitude*)&(gSinCosWeakAmpDataTest[rdcCore]));
    
    if(gSinCosWeakAmpDataTest [rdcCore].lowAmplitude_error == false)
    {
        testLog[rdcCore].SinCosWeakAmpData = TEST_FAIL;
        // return testStatus;
    }else
    {
        /* Reset Diagnostics */
        RDC_getDiagnosticsWeakAmplitudeData(base, rdcCore, (Diag_Mon_Sin_Cos_Weak_Amplitude *)&(gSinCosWeakAmpDataInit[rdcCore]));

        /* Clear Flags */
        uint32_t interruptSource = RDC_INTERRUPT_SOURCE_LOWAMPLITUDE_ERR;
                                
        RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
        testLog[rdcCore].SinCosWeakAmpData = TEST_PASS;
    }

    if(gMotorState == MOTOR_ROTATING)
    {
        /*
        * Checking the Sine and Cosine Gain Drift Diagnostics.
        * 
        * In order to trigger a fault, configuring the low threshold to smaller and high threshold to a higher value
        * 
        * Hence,
        * the High threshold can be 0.8*2^15 = 26214 and 
        * the Low Threshold can be 0.5*2^15  = 16384.
        * 
        */
        RDC_getDiagnosticsSinCosGainDriftData(base, rdcCore, (Diag_Mon_SinCos_Gain_drift_data *)&(gSinCosGainDriftDataTest[rdcCore]));
        gSinCosGainDriftDataTest [rdcCore].gain_drift_threshold_hi = (uint16_t) 30000;
        gSinCosGainDriftDataTest [rdcCore].gain_drift_threshold_lo = (uint16_t) 1000;
        gSinCosGainDriftDataTest [rdcCore].gain_drift_en     = false;

        RDC_setDiagnosticsSinCosGainDriftData(base, rdcCore, (Diag_Mon_SinCos_Gain_drift_data *)&(gSinCosGainDriftDataTest[rdcCore]));

        /* After every train limit rotations the sinsq values are accumulated and the gains and phase corrections are calculated */
        RDC_clearPhaseGainEstimationStatus(base, rdcCore);
        while(RDC_getPhaseGainEstimationStatus(base, rdcCore) != true);

        RDC_getDiagnosticsSinCosGainDriftData(base, rdcCore, (Diag_Mon_SinCos_Gain_drift_data *)&(gSinCosGainDriftDataTest[rdcCore]));
        if(
            (gSinCosGainDriftDataTest [rdcCore].gain_drift_cos_hi == false) ||
            (gSinCosGainDriftDataTest [rdcCore].gain_drift_cos_lo == false) ||
            (gSinCosGainDriftDataTest [rdcCore].gain_drift_sin_hi == false) ||
            (gSinCosGainDriftDataTest [rdcCore].gain_drift_sin_lo == false))
        {
            testLog[rdcCore].SinCosGainDriftData = TEST_FAIL;
            // return testStatus;
        }else
        {
            /* Reset Diagnostics */
            RDC_setDiagnosticsSinCosGainDriftData(base, rdcCore, (Diag_Mon_SinCos_Gain_drift_data *)&(gSinCosGainDriftDataInit[rdcCore]));

            /* Clear Flags */
            uint32_t interruptSource = RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_LO_ERR  |\
                                    RDC_INTERRUPT_SOURCE_GAINDRIFT_SIN_HI_ERR  |\
                                    RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_LO_ERR  |\
                                    RDC_INTERRUPT_SOURCE_GAINDRIFT_COS_HI_ERR ;
            RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
            testLog[rdcCore].SinCosGainDriftData = TEST_PASS;
        }

        /* 
         * Checking the Cosine Phase Drift Diagnostics.
         * 
         * To Trigger a fault the thresholds are configured to very small values
         * 
         * Hence,
         * High threshold can be of 1
         * Low threshold can be of -1 
         */
        RDC_getDiagnosticsCosPhaseDriftData(base, rdcCore, (Diag_Mon_Cos_Phase_drift_data *)&(gSinCosPhaseDriftDataTest[rdcCore]));
        gSinCosPhaseDriftDataTest [rdcCore].phase_drift_threshold_hi = 1;
        gSinCosPhaseDriftDataTest [rdcCore].phase_drift_threshold_lo = -1;
        gSinCosPhaseDriftDataTest [rdcCore].phase_drift_en = false;

        RDC_setDiagnosticsCosPhaseDriftData(base, rdcCore, (Diag_Mon_Cos_Phase_drift_data *)&(gSinCosPhaseDriftDataTest[rdcCore]));

        /* After every train limit rotations the sinsq values are accumulated and the gains and phase corrections are calculated */
        RDC_clearPhaseGainEstimationStatus(base, rdcCore);
        while(RDC_getPhaseGainEstimationStatus(base, rdcCore) != true);

        RDC_getDiagnosticsCosPhaseDriftData(base, rdcCore, (Diag_Mon_Cos_Phase_drift_data *)&(gSinCosPhaseDriftDataTest[rdcCore]));
        if(
            (gSinCosPhaseDriftDataTest [rdcCore].phase_drift_cos_hi  == false) ||
            (gSinCosPhaseDriftDataTest [rdcCore].phase_drift_cos_lo  == false))
        {
            testLog[rdcCore].SinCosPhaseDriftData = TEST_FAIL;
            // return testStatus;
        }else
        {
            /* Reset Diagnostics */
            RDC_setDiagnosticsCosPhaseDriftData(base, rdcCore, (Diag_Mon_Cos_Phase_drift_data *)&(gSinCosPhaseDriftDataInit[rdcCore]));

            /* Clear Flags */
            uint32_t interruptSource = RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_LO_ERR  |\
                                    RDC_INTERRUPT_SOURCE_PHASEDRIFT_COS_HI_ERR;

            RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
            testLog[rdcCore].SinCosPhaseDriftData = TEST_PASS;
        }
    
        /* 
        * Checking the Rotational Signal Integrity Check
        * 
        * rotpeak_level : the value over the +rotpeak_level is considered positive peak and 
        * below -rotpeak_level is considered negative peak. 
        * To trigger a fault, use the rotpeak_level to a high value like 2^15
        */
        RDC_getDiagnosticsRotationalSignalIntegrityData(base, rdcCore, (Diag_Mon_Rotational_Signal_Integrity_data *)&(gRotSigIntegrityDataTest[rdcCore]));
        gRotSigIntegrityDataTest [rdcCore].rotpeak_level    = 32000;
        gRotSigIntegrityDataTest [rdcCore].zero_cross_rot_en    = false;
        RDC_setDiagnosticsRotationalSignalIntegrityData(base, rdcCore, (Diag_Mon_Rotational_Signal_Integrity_data *)&(gRotSigIntegrityDataTest[rdcCore]));
        
        /* The Motor needs to rotate atleast once for this check */
        ClockP_usleep((uint32_t) ((float) 2.0/gMotorCurrentSpeed_rps));

        RDC_getDiagnosticsRotationalSignalIntegrityData(base, rdcCore, (Diag_Mon_Rotational_Signal_Integrity_data *)&(gRotSigIntegrityDataTest[rdcCore]));
        if(
            (gRotSigIntegrityDataTest [rdcCore].cos_neg_zc_peak_mismatch_err == false) ||
            (gRotSigIntegrityDataTest [rdcCore].cos_pos_zc_peak_mismatch_err == false) ||
            (gRotSigIntegrityDataTest [rdcCore].sin_neg_zc_peak_mismatch_err == false) ||
            (gRotSigIntegrityDataTest [rdcCore].sin_pos_zc_peak_mismatch_err == false) ||
            (gRotSigIntegrityDataTest [rdcCore].sin_multi_zc_error_err   == false) ||
            (gRotSigIntegrityDataTest [rdcCore].cos_multi_zc_error_err   == false))
        {
            testLog[rdcCore].RotSigIntegrityData = TEST_FAIL;
            // return testStatus;
        }else
        {
            /* Reset Diagnostics */
            RDC_setDiagnosticsRotationalSignalIntegrityData(base, rdcCore, (Diag_Mon_Rotational_Signal_Integrity_data *)&(gRotSigIntegrityDataInit[rdcCore]));

            /* Clear Flags */
            uint32_t interruptSource = RDC_INTERRUPT_SOURCE_COS_MULTI_ZC_ERROR_ERR    | \
                                    RDC_INTERRUPT_SOURCE_SIN_MULTI_ZC_ERROR_ERR       | \
                                    RDC_INTERRUPT_SOURCE_COS_NEG_ZC_PEAK_MISMATCH_ERR | \
                                    RDC_INTERRUPT_SOURCE_COS_POS_ZC_PEAK_MISMATCH_ERR | \
                                    RDC_INTERRUPT_SOURCE_SIN_NEG_ZC_PEAK_MISMATCH_ERR | \
                                    RDC_INTERRUPT_SOURCE_SIN_POS_ZC_PEAK_MISMATCH_ERR;
                                    
            RDC_clearCoreInterrupt(base, rdcCore, interruptSource);
            testLog[rdcCore].RotSigIntegrityData = TEST_PASS;
        }
    }

    if(
        (testLog[rdcCore].SinCosOffsetDriftData == TEST_FAIL) ||
        (testLog[rdcCore].ExcFreqDegData        == TEST_FAIL) ||
        (testLog[rdcCore].SigIntegritySinSqData == TEST_FAIL) ||
        (testLog[rdcCore].SinCosHighAmpData     == TEST_FAIL) ||
        (testLog[rdcCore].SinCosWeakAmpData     == TEST_FAIL) ||
        (testLog[rdcCore].SinCosGainDriftData   == TEST_FAIL) ||
        (testLog[rdcCore].SinCosPhaseDriftData  == TEST_FAIL) ||
        (testLog[rdcCore].RotSigIntegrityData   == TEST_FAIL)
        )
    {
        testStatus = SystemP_FAILURE;
    }

    return testStatus;
}

void App_motorRun(uint32_t motor_instance, float angle_pu, float velocity_rps)
{
    while(angle_pu > 1){
        angle_pu -= 1;
    }

    float angle_rad = angle_pu * 2 * PI_VAL;
    angle_rad = PI_VAL - angle_rad;

    float w_rps     = velocity_rps;

    /* Note : the following is merely a placeholder for motor control. 
    The application uses an overly simplified SVPWM generation to control the motor */

    /* Ramping the motor to a reference speed */
    float w_rpm     = w_rps*60;
    float w_err     = w_rpm - gMotorRefSpeed_rpm ;
    if (w_err < -gMotorSpeedStep_rpm){
        w_err = -gMotorSpeedStep_rpm;
    }else if (w_err > gMotorSpeedStep_rpm){
        w_err = gMotorSpeedStep_rpm;
    }
    float w_drive_rpm = w_rpm + w_err;
    float Vq = w_drive_rpm * MOTOR_KE_VALUE_PER_KRPM/1000;

    float u_alpha = cosf(angle_rad)*(0) - sinf(angle_rad)*(Vq);
    float u_beta  = sinf(angle_rad)*(0) + cosf(angle_rad)*(Vq);
    
    float dcBus_result = 12;
    float u_alpha_pu = u_alpha / dcBus_result;
    float u_beta_pu  = u_beta  / dcBus_result;

    float X = u_alpha_pu;
    float Y = -0.5*u_alpha_pu    +0.8660*u_beta_pu;
    float Z = -0.5*u_alpha_pu    -0.8660*u_beta_pu;
    
    float min_value;
    min_value = (X<Y) ? X : Y;
    min_value = (min_value < Z) ? min_value : Z;

    float max_value;
    max_value = (X>Y) ? X : Y;
    max_value = (max_value > Z) ? max_value : Z;

    float avg_value = (min_value + max_value)*0.5;

    float ta_on = (X - avg_value);
    float tb_on = (Y - avg_value);
    float tc_on = (Z - avg_value);

    ta_on = (ta_on > 0.5) ? 0.5 : (ta_on < -0.5) ? -0.5 : ta_on;
    tb_on = (tb_on > 0.5) ? 0.5 : (tb_on < -0.5) ? -0.5 : tb_on;
    tc_on = (tc_on > 0.5) ? 0.5 : (tc_on < -0.5) ? -0.5 : tc_on;

    ta_on += 0.5;
    tb_on += 0.5;
    tc_on += 0.5;

    uint32_t CMP_a_on = (uint32_t)(ta_on*gMotorPwmPeriodVal);
    uint32_t CMP_b_on = (uint32_t)(tb_on*gMotorPwmPeriodVal);
    uint32_t CMP_c_on = (uint32_t)(tc_on*gMotorPwmPeriodVal);

    CMP_a_on = (CMP_a_on <= 3) ? 3 : (CMP_a_on >= gMotorPwmPeriodVal) ? gMotorPwmPeriodVal - 3 : CMP_a_on; 
    CMP_b_on = (CMP_b_on <= 3) ? 3 : (CMP_b_on >= gMotorPwmPeriodVal) ? gMotorPwmPeriodVal - 3 : CMP_b_on; 
    CMP_c_on = (CMP_c_on <= 3) ? 3 : (CMP_c_on >= gMotorPwmPeriodVal) ? gMotorPwmPeriodVal - 3 : CMP_c_on; 

    if(motor_instance == MOTOR1)
    {
        EPWM_setCounterCompareValue(PWM_1A_EPMW0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_a_on);
        EPWM_setCounterCompareValue(PWM_1B_EPMW1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_b_on);
        EPWM_setCounterCompareValue(PWM_1C_EPMW2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_c_on);
    }else
    {
        EPWM_setCounterCompareValue(PWM_2A_EPMW4_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_a_on);
        EPWM_setCounterCompareValue(PWM_2B_EPMW5_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_b_on);
        EPWM_setCounterCompareValue(PWM_2C_EPMW6_BASE_ADDR, EPWM_COUNTER_COMPARE_A, CMP_c_on);
    }
    
}
