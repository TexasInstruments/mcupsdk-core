# EPWM valley switching {#EXAMPLES_DRIVERS_EPWM_VALLEY_SWITCHING}

[TOC]

# Introduction

EPWM Valley Switching

This example configures ePWMx as follows
 - ePWMx with DCAEVT1 forcing the ePWM output LOW
 - GPIO_A is used as the input to the INPUT XBAR INPUT1
 - INPUT1 (from INPUT XBAR) is used as the source for DCAEVT1
 - GPIO_B is set to output and toggled in the main loop to trip the PWM

 - ePWMx with DCBEVT1 forcing the ePWM output LOW
 - GPIO_A is used as the input to the INPUT XBAR INPUT1
 - INPUT1 (from INPUT XBAR) is used as the source for DCBEVT1
 - GPIO_B is set to output and toggled in the main loop to trip the PWM
 - DCBEVT1 uses the filtered version of DCBEVT1
 - The DCFILT signal uses the valley switching module to delay the
 - DCFILT signal by a software defined DELAY value.

\imageStyle{am263_epwm_valley_switching.png,width:60%}
 \image html am263_epwm_valley_switching.png "Block Diagram for EPWM Valley Switching example"

# External Connections

- For AM263x-CC
    - GPIO48 is connected to GPIO122.
    - EPWM0_A and EPWM0_B pin can be connected to an oscilloscope to view the waveform.
- For AM263x-LP
    - GPIO11 is connected to GPIO12.
    - EPWM1_A and EPWM1_B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC Pin 52 to HSEC Pin 72
- Capture waveform on HSEC Pin 49 for epwm0_A
- Capture waveform on HSEC Pin 51 for epwm0_B

## AM263X-LP
- Connect boosterpack header J1/J3 pin 7 to J2/J4 pin 18
- Capture waveform on boosterpack header J2/J4 Pin 37 for epwm1_A
- Capture waveform on boosterpack header J2/J4 Pin 38 for epwm1_B

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_VALLEY_SWITCHING_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_valley_switching

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
EPWM Valley Switching Test Started ...
EPWM Valley Switching Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_epwm_valley_switching_output.png,width:80%}
 \image html am263_epwm_valley_switching_output.png "EPWM Valley Switching waveform"

