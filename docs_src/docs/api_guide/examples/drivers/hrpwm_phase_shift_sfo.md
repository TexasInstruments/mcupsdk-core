# EPWM HR Phase Shift SFO {#EXAMPLES_DRIVERS_HRPWM_PHASE_SHIFT_SFO}

[TOC]

# Introduction

This example modifies the MEP control registers to show edge displacement for high-resolution phase shift with ePWM in Up count mode due to the HRPWM control extension of the respective ePWM module.

\imageStyle{am263_hrpwm_phase_shift_sfo_fig1.PNG,width:80%}
\image html am263_hrpwm_phase_shift_sfo_fig1.PNG "Block diagram"

# External Connections
- EPWM0_A, EPWM1_A pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC or AM263PX-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC 49 for epwm0_A
- Connect HSEC 53 for epwm1_A

## AM261X-LP
When using AM261x-LP
- Connect J5/J7 Pin 70 for epwm0_A
- Connect J5/J7 Pin 69 for epwm1_A

# Supported Combinations {#EXAMPLES_DRIVERS_HRPWM_PHASE_SHIFT_SFO_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/hrpwm_phase_shift_sfo/

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
EPWM High Resolution Phase Shift Test Started ...
Please observe pins HSEC 49 and HSEC 53
High Resolution Phase 14.39 degree. (shift = 20.000 nanoseconds)
EPWM1 PHS = 6, PHSHR = 0

Calibration... Running...

High Resolution Phase 14.54 degree. (shift = 20.208 nanoseconds)
EPWM1 PHS = 7, PHSHR = 246

Calibration... Running...

High Resolution Phase 14.69 degree. (shift = 20.416 nanoseconds)
EPWM1 PHS = 7, PHSHR = 235

Calibration... Running...

High Resolution Phase 14.84 degree. (shift = 20.624 nanoseconds)
EPWM1 PHS = 7, PHSHR = 225

Calibration... Running...

High Resolution Phase 14.99 degree. (shift = 20.833 nanoseconds)
EPWM1 PHS = 7, PHSHR = 214

Calibration... Complete
SFO status=1 , MEP_ScaleFactor=45

High Resolution Phase 15.14 degree. (shift = 21.041 nanoseconds)
EPWM1 PHS = 7, PHSHR = 203

Calibration... Running...

High Resolution Phase 15.29 degree. (shift = 21.249 nanoseconds)
EPWM1 PHS = 7, PHSHR = 193

\endcode

\imageStyle{am263_hrpwm_phase_shift_sfo_fig2.PNG,width:80%}
\image html am263_hrpwm_phase_shift_sfo_fig2.PNG "Expected output"
