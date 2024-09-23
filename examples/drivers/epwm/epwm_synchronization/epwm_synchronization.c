/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

/* # Example Description
 The example uses the following EPWMs to showcase the Synchronization features which includes:
 1. CONFIG_EPWM0 - This EPWM module is taken as the reference with 0 degree phase shift
 2. CONFIG_EPWM1 - This EPWM module is phase shifted by 300 TBCLK
 3. CONFIG_EPWM2 - This EPWM module is phase shifted by 600 TBCLK
 4. CONFIG_EPWM3 - This EPWM module is phase shifted by 900 TBCLK
 5. CONFIG_EPWM4 - This EPWM module is phase shifted by 450 TBCLK
 6. CONFIG_EPWM5 - This EPWM module is phase shifted by 750 TBCLK

 1. REF_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM0_A
 2. ECAP1_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM1_A
 3. ECAP2_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM2_A
 4. ECAP3_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM3_A
 5. ECAP4_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM4_A
 6. ECAP5_CAPTURE - This ECAP module instance is used to capture falling edge event of EPWM5_A


 # External Connections

When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
Probe the following on the HSEC pins
- CONFIG_EPWM0 - EPWM 0A/0B : 49 / 51
- CONFIG_EPWM1 - EPWM 1A/1B : 53 / 55
- CONFIG_EPWM2 - EPWM 2A/2B : 50 / 52
- CONFIG_EPWM3 - EPWM 3A/3B : 54 / 56
- CONFIG_EPWM4 - EPWM 4A/4B : 57 / 59
- CONFIG_EPWM5 - EPWM 5A/5B : 61 / 63

Early Access: For AM263Px-CC E1, the connections is same as that of AM263x

When using AM263X-LP
- CONFIG_EPWM0 - EPWM 0A/0B :   J2.11 / J6.59
- CONFIG_EPWM1 - EPWM 1A/1B :   J4.37 / J4.38
- CONFIG_EPWM2 - EPWM 2A/2B :   J4.39 / J4.40
- CONFIG_EPWM3 - EPWM 3A/3B :   J8.77 / J8.78
- CONFIG_EPWM4 - EPWM 9A/9B :   J8.75 / J8.76
- CONFIG_EPWM5 - EPWM 13A/13B : J8.79 / J8.80

When using AM261X-LP
- CONFIG_EPWM0 - EPWM 0A/0B :   J7.70 / J8.57
- CONFIG_EPWM1 - EPWM 1A/1B :   J7.69 / J7.63
- CONFIG_EPWM2 - EPWM 2A/2B :   J2.40 / J2.39
- CONFIG_EPWM3 - EPWM 3A/3B :   J2.38 / J2.37
- CONFIG_EPWM4 - EPWM 4A/4B :   J2.36 / J2.35
- CONFIG_EPWM5 - EPWM 6A/6B :   J6.78 / J6.77
*/


uint32_t gEpwm0BaseAddr = CONFIG_EPWM0_BASE_ADDR;
uint32_t gEpwm1BaseAddr = CONFIG_EPWM1_BASE_ADDR;
uint32_t gEpwm2BaseAddr = CONFIG_EPWM2_BASE_ADDR;
uint32_t gEpwm3BaseAddr = CONFIG_EPWM3_BASE_ADDR;



uint32_t gEcap0BaseAddr = REF_CAPTURE_BASE_ADDR;
uint32_t gEcap1BaseAddr = ECAP1_CAPTURE_BASE_ADDR;
uint32_t gEcap2BaseAddr = ECAP2_CAPTURE_BASE_ADDR;
uint32_t gEcap3BaseAddr = ECAP3_CAPTURE_BASE_ADDR;
uint32_t gEcap4BaseAddr = ECAP4_CAPTURE_BASE_ADDR;
uint32_t gEcap5BaseAddr = ECAP5_CAPTURE_BASE_ADDR;


#define RISING_EDGE_DELAY_1 300
#define RISING_EDGE_DELAY_2 600
#define RISING_EDGE_DELAY_3 900
#define RISING_EDGE_DELAY_4 450
#define RISING_EDGE_DELAY_5 750
#define SYSTEM_CLOCK        200000000
#define TIME_BASE_CLOCK     25000000
#define FREQUENCY           12500
#define PHASE_FULL          360

#define SCALING_FACTOR (8)

typedef struct{

    uint32_t                         epwmModule;
    uint16_t                         phaseCount;
    uint16_t                         compCountA;
    uint16_t                         compCountB;
    bool                             enableSOCA;
    bool                             enableSOCB;
    EPWM_SignalParams                signalParams;
    EPWM_SyncInPulseSource           source;
    EPWM_CounterCompareModule        compModule;
    EPWM_CounterCompareModule        counterCompareA;
    EPWM_CounterCompareModule        counterCompareB;

} EPWM_Details;

EPWM_Details gepwm4_details;
EPWM_Details gepwm5_details;
void AppEpwmInit(void* args);
void AppEpwmPhaseUpdate(EPWM_Details* epwmUnit, uint16_t phaseInDegree);
void AppEpwmDutyCycleUpdate(EPWM_Details* epwmUnit, Float32 dutyCycleA, Float32 dutyCycleB);

void epwm_synchronization_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    /* Initializing and setting configurations for EPWM4 and EPWM5 */
    AppEpwmInit(NULL);
    DebugP_log("\nEPWM Synchronization Test Started ...\r\n");
    DebugP_log("EPWM Synchronization Example runs for 5 Secs \r\n");


    ClockP_sleep(5);
    /* Capturing the time stamp for falling edge events of EPWM0, EPWM1, EPWM2 and EPWM3 waveforms using ECAP */
    volatile uint32_t fallingEdge_REF = (ECAP_getEventTimeStamp(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    volatile uint32_t fallingEdge_ECAP1 = (ECAP_getEventTimeStamp(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    volatile uint32_t fallingEdge_ECAP2 = (ECAP_getEventTimeStamp(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    volatile uint32_t fallingEdge_ECAP3 = (ECAP_getEventTimeStamp(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    volatile uint32_t fallingEdge_ECAP4 = (ECAP_getEventTimeStamp(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    volatile uint32_t fallingEdge_ECAP5 = (ECAP_getEventTimeStamp(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;

    // /* Function to update phase while the EPWMs are running */
    AppEpwmPhaseUpdate(&gepwm4_details, 90);
    AppEpwmPhaseUpdate(&gepwm5_details, 162);

    /* Function to update duty cycle while the EPWMs are running */
    AppEpwmDutyCycleUpdate(&gepwm4_details, 0.25, 0.75);

    DebugP_log("\tFalling Edge timestamp of Reference waveform  : \t%d\r\n",fallingEdge_REF );
    DebugP_log("\tFalling Edge timestamp of EPWM1 : \t%d\r\n",fallingEdge_ECAP1 );
    DebugP_log("\tFalling Edge timestamp of EPWM2 : \t%d\r\n",fallingEdge_ECAP2 );
    DebugP_log("\tFalling Edge timestamp of EPWM3 : \t%d\r\n",fallingEdge_ECAP3 );
    DebugP_log("\tFalling Edge timestamp of EPWM4 : \t%d\r\n",fallingEdge_ECAP4 );
    DebugP_log("\tFalling Edge timestamp of EPWM5 : \t%d\r\n",fallingEdge_ECAP5 );



    /* Calculating the delay of the reference EPWM waveform with EPWM1, EPWM2 and EPWM3 waveforms */
    uint32_t fallingEdgeDelay_EPWM1 = (fallingEdge_REF - fallingEdge_ECAP1);
    uint32_t fallingEdgeDelay_EPWM2 = (fallingEdge_REF - fallingEdge_ECAP2);
    uint32_t fallingEdgeDelay_EPWM3 = (fallingEdge_REF - fallingEdge_ECAP3);
    uint32_t fallingEdgeDelay_EPWM4 = (fallingEdge_REF - fallingEdge_ECAP4);
    uint32_t fallingEdgeDelay_EPWM5 = (fallingEdge_REF - fallingEdge_ECAP5);
    DebugP_log("\tObserved Phase Delay between Reference Waveform and EPWM1 Waveform : \t%d\r\n\tObserved Phase Delay between Reference Waveform and EPWM2 Waveform : \t%d\r\n\tObserved Phase Delay between Reference Waveform and EPWM3 Waveform : \t%d\r\n\tObserved Phase Delay between Reference Waveform and EPWM4 Waveform : \t%d\r\n\tObserved Phase Delay between Reference Waveform and EPWM5 Waveform : \t%d\r\n",fallingEdgeDelay_EPWM1, fallingEdgeDelay_EPWM2, fallingEdgeDelay_EPWM3, fallingEdgeDelay_EPWM4, fallingEdgeDelay_EPWM5);

    /* 
    When the Sync Pulse arrives, the rising edge is detected and the EPWM holds the state of the Sync singal present. 
    The phase shift value is loaded to the TB Counter of EPWM on the next valid TBCLK. 
    This may cause a TBCLK delay on the "Synced" PWM with the "Syncing" PWM. Hence the 1 count difference in the validation 
    */
    if(((RISING_EDGE_DELAY_1 - fallingEdgeDelay_EPWM1) <= 1) && ((RISING_EDGE_DELAY_2 - fallingEdgeDelay_EPWM2) <= 1) && ((RISING_EDGE_DELAY_3 - fallingEdgeDelay_EPWM3) <= 1) && ((RISING_EDGE_DELAY_4 - fallingEdgeDelay_EPWM4) <= 1) && ((RISING_EDGE_DELAY_5 - fallingEdgeDelay_EPWM5) <= 1))
    {
        DebugP_log("EPWM Synchronization Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("EPWM Synchronization Test Failed!!");
    }


    Board_driversClose();
    Drivers_close();

}

void AppEpwmInit(void *args)
{
    /* Setting the base address */
    gepwm4_details.epwmModule = CONFIG_EPWM4_BASE_ADDR;
    gepwm5_details.epwmModule = CONFIG_EPWM5_BASE_ADDR;

    /* Setting Frequency and Duty Cycle for EPWM4 */
    gepwm4_details.signalParams.freqInHz = FREQUENCY;
    gepwm4_details.signalParams.dutyValA = 0.50;
    gepwm4_details.signalParams.dutyValB = 0.25;
    gepwm4_details.signalParams.sysClkInHz = SYSTEM_CLOCK;
    gepwm4_details.signalParams.tbCtrMode = EPWM_COUNTER_MODE_UP;
    gepwm4_details.signalParams.tbClkDiv = EPWM_CLOCK_DIVIDER_8 ;
    gepwm4_details.signalParams.tbHSClkDiv = EPWM_HSCLOCK_DIVIDER_1;
    gepwm4_details.signalParams.invertSignalB = false;

    EPWM_configureSignal(gepwm4_details.epwmModule, &gepwm4_details.signalParams);

    /* Setting Frequency and Duty Cycle for EPWM5 */
    gepwm5_details.signalParams.freqInHz = FREQUENCY;
    gepwm5_details.signalParams.dutyValA = 0.50;
    gepwm5_details.signalParams.dutyValB = 0.25;
    gepwm5_details.signalParams.sysClkInHz = SYSTEM_CLOCK;
    gepwm5_details.signalParams.tbCtrMode = EPWM_COUNTER_MODE_UP;
    gepwm5_details.signalParams.tbClkDiv = EPWM_CLOCK_DIVIDER_8 ;
    gepwm5_details.signalParams.tbHSClkDiv = EPWM_HSCLOCK_DIVIDER_1;
    gepwm5_details.signalParams.invertSignalB = false;

    EPWM_configureSignal(gepwm5_details.epwmModule, &gepwm5_details.signalParams);

    /* Setting Sync In Pulse Source for EPWM4 */
    /* Sync In pulse source for EPWM4 is EPWM0 sync-out signal, and EPWM4 does not have any sync-out pulse */
    gepwm4_details.source = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0;
    // EPWM_enableSyncOutPulseSource(gepwm4_details.epwmModule, 0);
    EPWM_setSyncInPulseSource(gepwm4_details.epwmModule, gepwm4_details.source);

    /* Setting Sync In Pulse Source for EPWM5 */
    /* Sync In pulse source for EPWM4 is EPWM0 sync-out signal, and EPWM5 does not have any sync-out pulse */
    gepwm5_details.source = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0;
    // EPWM_enableSyncOutPulseSource(gepwm5_details.epwmModule, 0);
    EPWM_setSyncInPulseSource(gepwm5_details.epwmModule, gepwm5_details.source);


    /* Setting Phase Shift Value for EPWM4 */
    /* The phase value is taken in degrees and is converted into timebase clock unit, after which it is given to the setPhaseShift function */
    /* Here 81 degrees corresponds to 450 Timebase clocks */
    gepwm4_details.phaseCount = (Float32)(((Float32)(81 % PHASE_FULL) / PHASE_FULL) * (uint16_t)(TIME_BASE_CLOCK/FREQUENCY));
    EPWM_enablePhaseShiftLoad(gepwm4_details.epwmModule);
    EPWM_setPhaseShift(gepwm4_details.epwmModule, gepwm4_details.phaseCount);

    /* Setting Phase Shift Value for EPWM5 */
    /* The phase value is taken in degrees and is converted into timebase clock unit, after which it is given to the setPhaseShift function */
    /* Here 135 degrees corresponds to 750 Timebase clocks */
    gepwm5_details.phaseCount = (Float32)(((Float32)(135 % PHASE_FULL) / PHASE_FULL) * (uint16_t)(TIME_BASE_CLOCK/FREQUENCY));
    EPWM_enablePhaseShiftLoad(gepwm5_details.epwmModule);
    EPWM_setPhaseShift(gepwm5_details.epwmModule, gepwm5_details.phaseCount);

     /* Enabling/Disabling ADC SOC Conversion for EPWM4 based of enableSOCA bool value */
    gepwm4_details.enableSOCA = false;
    gepwm4_details.enableSOCB = false;
    if(gepwm4_details.enableSOCA == true)
    {
        EPWM_enableADCTrigger(gepwm4_details.epwmModule, EPWM_SOC_A);
        EPWM_setADCTriggerSource(gepwm4_details.epwmModule, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, 0);
	    EPWM_setADCTriggerEventPrescale(gepwm4_details.epwmModule, EPWM_SOC_A, 1);
	    EPWM_disableADCTriggerEventCountInit(gepwm4_details.epwmModule, EPWM_SOC_A);
	    EPWM_setADCTriggerEventCountInitValue(gepwm4_details.epwmModule, EPWM_SOC_A, 0);
    }
    else
    {
       EPWM_disableADCTrigger(gepwm4_details.epwmModule, EPWM_SOC_A);
    }

     /* Enabling/Disabling ADC SOC Conversion for EPWM4 based of enableSOCB bool value */
    if(gepwm4_details.enableSOCB == true)
    {
        EPWM_enableADCTrigger(gepwm4_details.epwmModule, EPWM_SOC_B);
        EPWM_setADCTriggerSource(gepwm4_details.epwmModule, EPWM_SOC_B, EPWM_SOC_TBCTR_ZERO, 0);
	    EPWM_setADCTriggerEventPrescale(gepwm4_details.epwmModule, EPWM_SOC_B, 1);
	    EPWM_disableADCTriggerEventCountInit(gepwm4_details.epwmModule, EPWM_SOC_B);
	    EPWM_setADCTriggerEventCountInitValue(gepwm4_details.epwmModule, EPWM_SOC_B, 0);

    }
    else
    {
       EPWM_disableADCTrigger(gepwm4_details.epwmModule, EPWM_SOC_B);
    }


     /* Enabling/Disabling ADC SOC Conversion for EPWM5 based of enableSOCA bool value */
    gepwm5_details.enableSOCA = false;
    gepwm5_details.enableSOCB = false;
    if(gepwm5_details.enableSOCA == true)
    {
        EPWM_enableADCTrigger(gepwm5_details.epwmModule, EPWM_SOC_A);
        EPWM_setADCTriggerSource(gepwm5_details.epwmModule, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, 0);
	    EPWM_setADCTriggerEventPrescale(gepwm5_details.epwmModule, EPWM_SOC_A, 1);
	    EPWM_disableADCTriggerEventCountInit(gepwm5_details.epwmModule, EPWM_SOC_A);
	    EPWM_setADCTriggerEventCountInitValue(gepwm5_details.epwmModule, EPWM_SOC_A, 0);

    }
    else
    {
       EPWM_disableADCTrigger(gepwm5_details.epwmModule, EPWM_SOC_A);
    }

     /* Enabling/Disabling ADC SOC Conversion for EPWM5 based of enableSOCA bool value */
    if(gepwm5_details.enableSOCB == true)
    {
        EPWM_enableADCTrigger(gepwm5_details.epwmModule, EPWM_SOC_B);
        EPWM_setADCTriggerSource(gepwm5_details.epwmModule, EPWM_SOC_B, EPWM_SOC_TBCTR_ZERO, 0);
	    EPWM_setADCTriggerEventPrescale(gepwm5_details.epwmModule, EPWM_SOC_B, 1);
	    EPWM_disableADCTriggerEventCountInit(gepwm5_details.epwmModule, EPWM_SOC_B);
	    EPWM_setADCTriggerEventCountInitValue(gepwm5_details.epwmModule, EPWM_SOC_B, 0);

    }
    else
    {
       EPWM_disableADCTrigger(gepwm5_details.epwmModule, EPWM_SOC_B);
    }


}

void AppEpwmPhaseUpdate(EPWM_Details* epwmUnit, uint16_t phaseInDegree)
{

    /* This function is used to update the phase value of running EPWMs */
    EPWM_setPhaseShift(epwmUnit->epwmModule, (Float32)(((Float32)(phaseInDegree % PHASE_FULL) / PHASE_FULL) * (uint16_t)(TIME_BASE_CLOCK/FREQUENCY)));

}
void AppEpwmDutyCycleUpdate(EPWM_Details* epwmUnit, Float32 dutyCycleA, Float32 dutyCycleB)
{

    /* This function is used to update the duty cycle of running EPWMs */
    epwmUnit->compModule = EPWM_COUNTER_COMPARE_A;
    uint16_t compCountA = (uint16_t)(dutyCycleA * (uint16_t)(TIME_BASE_CLOCK/FREQUENCY));
    EPWM_setCounterCompareValue(epwmUnit->epwmModule, epwmUnit->compModule, compCountA);

    epwmUnit->compModule = EPWM_COUNTER_COMPARE_B;
    uint16_t compCountB = (uint16_t)(dutyCycleB * (uint16_t)(TIME_BASE_CLOCK/FREQUENCY));
    EPWM_setCounterCompareValue(epwmUnit->epwmModule, epwmUnit->compModule, compCountB);

}
