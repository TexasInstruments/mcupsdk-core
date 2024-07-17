# EPWM UP-COUNT ACTION QUALIFIER {#EXAMPLES_DRIVERS_EPWM_UP_AQ}

[TOC]

# Introduction

This example configures ePWM0, ePWM1 and ePWM2 to produce an waveform with independent modulation on ePWMxA and ePWMxB. The TB counter is in up count mode for this example.

The compare values CMPA and CMPB are modified within the ePWM's ISR.

## ISR Configuration
1. INTXbar0, INTXbar1 and INTXbar2 are set for EPWM0INT, EPWM1INT and EPWM2INT
2. App_epwmIntrISR_x service this interrupts
    - Updates the CMPA and CMPB compare values at every 10'th interrupt as follows:
        - If CMPA/B is incrising, check if it reached max value. If not, increase CMPA/B else, change direction and decrease CMPA/B
        - If CMPA/B is decrising, check if it reached min value. If not, decrease CMPA/B else, change direction and increase CMPA/B

\imageStyle{am263_epwm_up_aq.png,width:50%}
\image html am263_epwm_up_aq.png "EPWM Asymetric Waveform with Independent Modulation using UP-Count mode"

# External Connections

- For AM263X-CC or AM263PX-CC
    - EPWM0_A and EPWM0_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM1_A and EPWM1_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM2_A and EPWM2_B pin can be connected to an oscilloscope to view the waveform.

- For AM263X-LP or AM263PX-LP
    - EPWM0_A and EPWM0_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM1_A and EPWM1_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM2_A and EPWM2_B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC or AM263PX-CC
When using AM263X-CC or AM263PX-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 49 for epwm0_A
- Capture waveform on HSEC Pin 51 for epwm0_B
- Capture waveform on HSEC Pin 53 for epwm1_A
- Capture waveform on HSEC Pin 55 for epwm1_B
- Capture waveform on HSEC Pin 50 for epwm2_A
- Capture waveform on HSEC Pin 52 for epwm2_B

## AM263X-LP or AM263PX-LP
- Capture waveform on boosterpack header J6/J8 Pin 11 for epwm0_A
- Capture waveform on boosterpack header J6/J8 Pin 59 for epwm0_B
- Capture waveform on boosterpack header J2/J4 Pin 37 for epwm1_A
- Capture waveform on boosterpack header J2/J4 Pin 38 for epwm1_B
- Capture waveform on boosterpack header J2/J4 Pin 39 for epwm2_A
- Capture waveform on boosterpack header J2/J4 Pin 40 for epwm2_B

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_UP_AQ_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_xcmp_multiple_edges

\endcond



# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM Action Qualifier Module Test Started ...
EPWM Action Qualifier using UP-Count mode Example runs for 30 Secs
EPWM Action Qualifier Module Test Passed!!
All tests have passed!!
\endcode


