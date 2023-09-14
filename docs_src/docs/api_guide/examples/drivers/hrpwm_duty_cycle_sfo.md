# EPWM HR duty cycle SFO {#EXAMPLES_DRIVERS_HRPWM_DUTY_CYCLE_SFO}

[TOC]

# Introduction

This example modifies the MEP control registers to show edge displacement for high-resolution duty cycle with ePWM in Up count mode due to the HRPWM control extension of the respective ePWM module.

\imageStyle{am263_hrpwm_duty_cycle_sfo_fig1.PNG,width:80%}
\image html am263_hrpwm_duty_cycle_sfo_fig1.PNG "Block diagram"

# External Connections
- EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC 49 for epwm0_A
- Connect HSEC 51 for epwm0_B


# Supported Combinations {#EXAMPLES_DRIVERS_HRPWM_DUTY_CYCLE_SFO_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/hrpwm_duty_cycle_sfo/

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
EPWM High Resolution Duty Cycle Test Started ...
Please observe pins HSEC 49 and HSEC 51.
High Resolution duty cycle 4.00 percent. (positive width = 20.000 nanoseconds)
EPWM0 CMP = 4, CMPHR = 0

Calibration... Running...

High Resolution duty cycle 4.15 percent. (positive width = 20.750 nanoseconds)
EPWM0 CMP = 4, CMPHR = 38

Calibration... Running...

High Resolution duty cycle 4.30 percent. (positive width = 21.500 nanoseconds)
EPWM0 CMP = 4, CMPHR = 76

Calibration... Running...

High Resolution duty cycle 4.45 percent. (positive width = 22.250 nanoseconds)
EPWM0 CMP = 4, CMPHR = 115

Calibration... Running...

High Resolution duty cycle 4.60 percent. (positive width = 23.000 nanoseconds)
EPWM0 CMP = 4, CMPHR = 153

Calibration... Complete
SFO status=1 , MEP_ScaleFactor=45

High Resolution duty cycle 4.75 percent. (positive width = 23.750 nanoseconds)
EPWM0 CMP = 4, CMPHR = 192

Calibration... Running...

High Resolution duty cycle 4.90 percent. (positive width = 24.500 nanoseconds)
EPWM0 CMP = 4, CMPHR = 230

…

\endcode

\imageStyle{am263_hrpwm_duty_cycle_sfo_fig2.PNG,width:80%}
\image html am263_hrpwm_duty_cycle_sfo_fig2.PNG "Expected output"
