
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
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description :
 *      The Example sets up the EPMWs(0,1,2,3,13,15) to showcase the Deadband
 * features, as follows,
 *      1. CONFIG_EPWM0 : Deadband disabled (Reference)
 *      2. CONFIG_EPWM1 : Deadband Active High
 *      3. CONFIG_EPWM2 : Deadband Active Low
 *      4. CONFIG_EPWM3 : Deadband Active High Complimentary
 *      5. CONFIG_EPWM4 : Deadband Active Low Complimentary
 *      6. CONFIG_EPWM5 : Deadband Output Swap i.e., (switch A and B outputs)
 *      7. CONFIG_EPWM6 : Deadband On Rising and Falling edges.
 *      8. SYNC_PWM     : To sync all the EPWMs and ECAPs for validation
 *
 * Note :
 *      - All the EPWM in the example are synced with CONFIG_EPWM0
 *        as its Counter reaches 0 and all EPMWs start counting up from 0 when
 *        sync pulse reaches.
 *
 * Watch Variables :
 *      - AM263x - CC :
 *         Probe the following
 *         - CONFIG_EPWM0 output on HSEC PIN 49  (EPWM0_A)
 *         - CONFIG_EPWM0 output on HSEC PIN 51  (EPWM0_B)
 *         - CONFIG_EPWM1 output on HSEC PIN 53  (EPWM1_A)
 *         - CONFIG_EPWM1 output on HSEC PIN 55  (EPWM1_B)
 *         - CONFIG_EPWM2 output on HSEC PIN 50  (EPWM2_A)
 *         - CONFIG_EPWM2 output on HSEC PIN 52  (EPWM2_B)
 *         - CONFIG_EPWM3 output on HSEC PIN 54  (EPWM3_A)
 *         - CONFIG_EPWM3 output on HSEC PIN 56  (EPWM3_B)
 *         - CONFIG_EPWM4 output on HSEC PIN 153 (EPWM13_A)
 *         - CONFIG_EPWM4 output on HSEC PIN 154 (EPWM13_B)
 *         - CONFIG_EPWM5 output on HSEC PIN 159 (EPWM15_A)
 *         - CONFIG_EPWM5 output on HSEC PIN 160 (EPWM15_B)
 *         - CONFIG_EPWM6 output on HSEC PIN 57  (EPWM4_A)
 *         - CONFIG_EPWM6 output on HSEC PIN 59  (EPWM4_B)
 *
 *      - AM263x - LP :
 *         Probe the following
 *         - CONFIG_EPWM0 output on  PIN J2/4 11  (EPWM0_A)
 *         - CONFIG_EPWM0 output on  PIN J6/8 59  (EPWM0_B)
 *         - CONFIG_EPWM1 output on  PIN J2/4 37  (EPWM1_A)
 *         - CONFIG_EPWM1 output on  PIN J2/4 38  (EPWM1_B)
 *         - CONFIG_EPWM2 output on  PIN J2/4 39  (EPWM2_A)
 *         - CONFIG_EPWM2 output on  PIN J2/4 40  (EPWM2_B)
 *         - CONFIG_EPWM3 output on  PIN J6/8 77  (EPWM3_A)
 *         - CONFIG_EPWM3 output on  PIN J6/8 78  (EPWM3_B)
 *         - CONFIG_EPWM4 output on  PIN J6/8 79  (EPWM13_A)
 *         - CONFIG_EPWM4 output on  PIN J6/8 80  (EPWM13_B)
 *         - CONFIG_EPWM5 output on  PIN J2/4 34  (EPWM15_A)
 *         - CONFIG_EPWM5 output on  PIN J5/7 45  (EPWM15_B)
 *         - CONFIG_EPWM6 output on  PIN J2/4 35  (EPWM14_A)
 *         - CONFIG_EPWM6 output on  PIN J2/4 36  (EPWM14_B)
 */

/* Get Address of ePWM */
uint32_t gEpwm0BaseAddr = CONFIG_EPWM0_BASE_ADDR;
uint32_t gEpwm1BaseAddr = CONFIG_EPWM1_BASE_ADDR;
uint32_t gEpwm2BaseAddr = CONFIG_EPWM2_BASE_ADDR;
uint32_t gEpwm3BaseAddr = CONFIG_EPWM3_BASE_ADDR;
uint32_t gEpwm4BaseAddr = CONFIG_EPWM4_BASE_ADDR;
uint32_t gEpwm5BaseAddr = CONFIG_EPWM5_BASE_ADDR;
uint32_t gEpwm6BaseAddr = CONFIG_EPWM6_BASE_ADDR;

/* Get Address of eCAP*/
uint32_t gEcap0BaseAddr = REF_CAPTURE_BASE_ADDR;
uint32_t gEcap1BaseAddr = AH_A_CAPTURE_BASE_ADDR;
uint32_t gEcap2BaseAddr = AH_B_CAPTURE_BASE_ADDR;

/* Syscfg values in RED and FED */
#define RISING_EDGE_DELAY 400
#define FALLING_EDGE_DELAY 200

/* EPWM Counter is prescaled by 16. ECAP counter is prescaled by 1*/
#define SCALING_FACTOR (16)



void epwm_deadband_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM DeadBand Test Started ...\r\n");
    DebugP_log("EPWM DeadBand Example runs for 5 Secs \r\n");

    /* Start all the EPWM Timebase Counters */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncDisableMask, TRUE);

    ClockP_sleep(5);
    /* Rising and Falling edge timestamps for reference waveform*/
    volatile uint32_t risingEdge_REF  = (ECAP_getEventTimeStamp(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;
    volatile uint32_t fallingEdge_REF = (ECAP_getEventTimeStamp(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_2))/SCALING_FACTOR;

    /* Rising and Falling edge timestamps for Active High A waveform*/
    volatile uint32_t risingEdge_AHA  = (ECAP_getEventTimeStamp(AH_A_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;
    volatile uint32_t fallingEdge_AHA = (ECAP_getEventTimeStamp(AH_A_CAPTURE_BASE_ADDR,ECAP_EVENT_2))/SCALING_FACTOR;

    /* Rising and Falling edge timestamps for Active High B waveform*/
    volatile uint32_t risingEdge_AHB  = (ECAP_getEventTimeStamp(AH_B_CAPTURE_BASE_ADDR,ECAP_EVENT_1))/SCALING_FACTOR;
    volatile uint32_t fallingEdge_AHB = (ECAP_getEventTimeStamp(AH_B_CAPTURE_BASE_ADDR,ECAP_EVENT_2))/SCALING_FACTOR;

    /* Stop all the EPMW Timebase Counters */
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncDisableMask, FALSE);

    DebugP_log("\tRising Edge timestamp of Reference waveform  : \t\t\t%d\r\n",risingEdge_REF );
    DebugP_log("\tFalling Edge timestamp of Reference waveform : \t\t\t%d\r\n",fallingEdge_REF);

    DebugP_log("\tRising Edge timestamp of Deadband Active High output A  : \t%d \t(Reference Rising Edge Timestamp + Rising Edge Delay)\r\n",risingEdge_AHA );
    DebugP_log("\tFalling Edge timestamp of Deadband Active High output A : \t%d \t(Same as Reference Falling Edge Timestamp)\r\n",fallingEdge_AHA);
    
    DebugP_log("\tRising Edge timestamp of Deadband Active High output B  : \t%d \t(Same as Reference Rising Edge Timestamp)\r\n",risingEdge_AHB );
    DebugP_log("\tFalling Edge timestamp of Deadband Active High output B : \t%d \t(Reference Falling Edge Timestamp + Falling Edge Delay)\r\n",fallingEdge_AHB);

    uint32_t risingEdgeDelay_AHA = (risingEdge_AHA - risingEdge_REF);
    uint32_t fallingEdgeDelay_AHB = (fallingEdge_AHB - fallingEdge_REF);

    DebugP_log("\tObserved Rising Edege Delay : \t%d\r\n\tObserved Falling Edege Delay : \t%d\r\n", risingEdgeDelay_AHA, fallingEdgeDelay_AHB);

    if( (risingEdge_REF == risingEdge_AHB)          &&
        (fallingEdge_REF == fallingEdge_AHA)        &&
        (risingEdgeDelay_AHA ==  RISING_EDGE_DELAY) &&
        (fallingEdgeDelay_AHB == FALLING_EDGE_DELAY))
    {
        DebugP_log("EPWM DeadBand Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("EPWM DeadBand Test Failed!!");
    }

    Board_driversClose();
    Drivers_close();
}
