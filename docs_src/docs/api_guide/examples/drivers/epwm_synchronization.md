# EPWM Synchronization {#EXAMPLES_DRIVERS_EPWM_SYNCHRONIZATION}

[TOC]

# Introduction

This example configures 6 instances of EPWM, with EPWM0 taken as the reference and the other
five EPWM instances are phase shifted with respect to it. EPWM0, EPWM1, EPWM2 and EPWM3 are configured
using Sysconfig while EPWM4 and EPWM5 are configured using a function. Additionally, functions are defined
that can change the phase and duty cycle of EPWMs while they are running. In this example synchronization is achieved by providing the
sync-out pulse of EPWM0 as the sync-in pulse to EPWM1, EPWM2, EPWM3, EPWM4 and EPWM5. On receiving this sync-in pulse
the EPWM time-base counter will start counting from the phase shift value up until period value. Further, 6 ECAP
modules are integrated in capture mode to capture the waveform in order to validate the synchronization results.
These ECAP modules are also synchronized wherein the sync-in pulse are provided by the sync-out pulse of EPWM0, and they are
configured to detect the event of falling edge in the EPWM waveforms

The sync-in options available are:
 1. EPWMx sync-out signal where x=0,1,2..23
 2. ECAPx sync-out signal where x=0,1,2..9
 3. Input XBAR outx signal where x=4,20
 4. TimeSync xbar pwm output x signal where x=0,1
 5. FSIRX0 Trigger x signal x=0,1,2,3
 6. FSIRX1 Trigger x signal x=0,1,2,3
 7. FSIRX2 Trigger x signal x=0,1,2,3
 8. FSIRX3 Trigger x signal x=0,1,2,3

The sync-out event options available are:
 1. Software force generated EPWM sync-out pulse
 2. Counter zero event
 3. Counter equal to CMPB event
 4. Counter equal to CMPC event
 5. Counter equal to CMPD event
 6. DCA Event 1 Sync signal generates EPWM sync-out pulse
 7. DCB Event 1 Sync signal generates EPWM sync-out pulse
 8. Combination of one or more of above sources

 For this example, we have configured EPWM0 sync-out signal to be the sync-in pulse for EPWM1, EPWM2, EPWM3, EPWM4, EPWM5. This sync-out pulse
 is generated at the counter zero event of EPWM0. Once the other modules receive this sync-in pulse, their counter will be
 at the phase value configured which is 300 for EPWM1, 600 for EPWM2, 900 for EPWM3, 450 for EPWM4, 750 for EPWM5. Also the six ECAP modules are synchronized
 with EPWM0, by configuring their sync-in source as EPWM0 sync-out signal. The phase values for all the ECAP modules are configured to be 0.
 In this example, the configurations for EPWM0, EPWM1, EPWM2 and EPWM3 are done through SysConfig while the configurations for EPWM4 and EPWM5 are done through
 the function AppEpwmInit in the C file. Further two more functions namely AppEpwmDutyCycleUpdate and AppEpwmPhaseUpdate are defined to update the duty cycle and
 phase of already running PWM waveforms.
 Here, the load from Shadow to Active registers occur when the time-base counter reaches zero. This event varies depending on the counter-mode we have set for our EPWM
 modules. If the counter mode is up-count/down-count then this event occurs only once. When the counter mode is up-down count then this event occurs two times.

\imageStyle{am263_epwm_synchronization_timebase_waveform.png,width:60%}
 \image html am263_epwm_synchronization_timebase_waveform.png "Waveform representation of phase shifted EPWM"

 # Example Description
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

\imageStyle{am263_epwm_synchronization_diagram.png,width:40%}
 \image html am263_epwm_synchronization_diagram.png "Block diagram of EPWM Synchronization"

\imageStyle{am263_epwm_synchronization_ecap_integration.png,width:40%}
 \image html am263_epwm_synchronization_ecap_integration.png "Integration of ECAP to capture EPWM waveform"

 # External Connections

When using AM263x-CC or AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
Probe the following
- CONFIG_EPWM0 output on HSEC PIN 49  (EPWM0_A)
- CONFIG_EPWM0 output on HSEC PIN 51  (EPWM0_B)
- CONFIG_EPWM1 output on HSEC PIN 53  (EPWM1_A)
- CONFIG_EPWM1 output on HSEC PIN 55  (EPWM1_B)
- CONFIG_EPWM2 output on HSEC PIN 50  (EPWM2_A)
- CONFIG_EPWM2 output on HSEC PIN 52  (EPWM2_B)
- CONFIG_EPWM3 output on HSEC PIN 54  (EPWM3_A)
- CONFIG_EPWM3 output on HSEC PIN 56  (EPWM3_B)
- CONFIG_EPWM4 output on HSEC PIN 57  (EPWM4_A)
- CONFIG_EPWM4 output on HSEC PIN 59  (EPWM4_B)
- CONFIG_EPWM5 output on HSEC PIN 61  (EPWM5_A)
- CONFIG_EPWM5 output on HSEC PIN 63  (EPWM5_B)


Early Access: For AM263Px-CC E1, the connections is same as that of AM263x

When using AM263X-LP or AM263PX-LP
- CONFIG_EPWM0 output on J2.11  (EPWM0_A)
- CONFIG_EPWM0 output on J6.59  (EPWM0_B)
- CONFIG_EPWM1 output on J4.37  (EPWM1_A)
- CONFIG_EPWM1 output on J4.38  (EPWM1_B)
- CONFIG_EPWM2 output on J4.39  (EPWM2_A)
- CONFIG_EPWM2 output on J4.40  (EPWM2_B)
- CONFIG_EPWM3 output on J8.77  (EPWM3_A)
- CONFIG_EPWM3 output on J8.78  (EPWM3_B)
- CONFIG_EPWM4 output on J8.75  (EPWM9_A)
- CONFIG_EPWM4 output on J8.76  (EPWM9_B)
- CONFIG_EPWM5 output on J8.79  (EPWM13_A)
- CONFIG_EPWM5 output on J8.80  (EPWM13_B)

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_SYNCHRONIZATION_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_synchronization

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM Synchronization Test Started ...
EPWM Synchronization Example runs for 5 Secs
	Falling Edge timestamp of Reference waveform  : 	1002
	Falling Edge timestamp of EPWM1 : 	702
	Falling Edge timestamp of EPWM2 : 	402
	Falling Edge timestamp of EPWM3 : 	102
	Falling Edge timestamp of EPWM4 : 	552
	Falling Edge timestamp of EPWM5 : 	252
	Observed Phase Delay between Reference Waveform and EPWM1 Waveform : 	300
	Observed Phase Delay between Reference Waveform and EPWM2 Waveform : 	600
	Observed Phase Delay between Reference Waveform and EPWM3 Waveform : 	900
	Observed Phase Delay between Reference Waveform and EPWM4 Waveform : 	450
	Observed Phase Delay between Reference Waveform and EPWM5 Waveform : 	750
EPWM Synchronization Test Passed!!
All tests have passed!!
\endcode


 \imageStyle{am263_epwm_synchronization_output_waveform.png,width:80%}
  \image html am263_epwm_synchronization_output_waveform.png "EPWM Synchronization Output Waveform"