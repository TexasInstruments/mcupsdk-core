# EPWM XCMP Multiple Edges {#EXAMPLES_DRIVERS_EPWM_XCMP_MULTIPLE_EDGES}

[TOC]

# Introduction

EPWM XCMP Multiple Edges

This example uses the ePWM module to generate multiple edges in a pwm cycle.
 - Three instances of epwm have been used to demonstrate the use of XCMP feature.
 - For EPWMx, XCMP feature is disabled and it generates waves with duty cycle of 37.5%
 - For EPWMy, XCMP feature is enabled and the only the ACTIVE set registers are used, duty cycle is 1%.
 - For EPWMz, XCMP feature is enabled and the ACTIVE as well as the three SHADOW set registers are used.
    - Using shadow set 3, waves with duty cycle 50% is generated for 5 cycles for both output channels.
    - Using shadow set 2, waves with duty cycle 0% is generated for 8 cycles for both output channels.
    - Using shadow set 1, waves with duty cycle 20% for channel A and 40% for channel B is generated and this continues.

# External Connections

- For AM263x-CC
    - EPWM0_A and EPWM0_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM1_A and EPWM1_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM2_A and EPWM2_B pin can be connected to an oscilloscope to view the waveform.

- For AM263x-LP
    - EPWM1_A and EPWM1_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM2_A and EPWM2_B pin can be connected to an oscilloscope to view the waveform.
    - EPWM3_A and EPWM3_B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 49 for epwm0_A
- Capture waveform on HSEC Pin 51 for epwm0_B
- Capture waveform on HSEC Pin 53 for epwm1_A
- Capture waveform on HSEC Pin 55 for epwm1_B
- Capture waveform on HSEC Pin 50 for epwm2_A
- Capture waveform on HSEC Pin 52 for epwm2_B

## AM263X-LP
- Capture waveform on boosterpack header J2/J4 Pin 37 for epwm1_A
- Capture waveform on boosterpack header J2/J4 Pin 38 for epwm1_B
- Capture waveform on boosterpack header J2/J4 Pin 39 for epwm2_A
- Capture waveform on boosterpack header J2/J4 Pin 40 for epwm2_B
- Capture waveform on boosterpack header J6/J8 Pin 77 for epwm3_A
- Capture waveform on boosterpack header J6/J8 Pin 78 for epwm3_B

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_XCMP_MULTIPLE_EDGES_COMBOS}

\cond SOC_AM263X

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
EPWM XCMP multiple edges Test Started ...
EPWM XCMP multiple edges Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_epwm_xcmp_multiple_edges_output.PNG,width:80%}
 \image html am263_epwm_xcmp_multiple_edges_output.PNG "EPWM XCMP multiple edges waveform"

