# EPWM HR duty cycle {#EXAMPLES_DRIVERS_HRPWM_DEADBAND_SFO}

[TOC]

# Introduction

This example modifies the MEP control registers to show edge displacement for high-resolution deadband with ePWM in Up count mode due to the HRPWM control extension of the respective ePWM module.

# External Connections
- EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC 49 for epwm0_A
- Connect HSEC 51 for epwm0_A



# Supported Combinations {#EXAMPLES_DRIVERS_HRPWM_DEADBAND_SFO_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/hrpwm_deadband_sfo/

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
EPWM High Resolution Deadband Test Started ...
Please observe pins HSEC 49 and HSEC 51.
\endcode

