# EPWM HR Deadband SFO {#EXAMPLES_DRIVERS_HRPWM_DEADBAND_SFO}

[TOC]

# Introduction

This example modifies the MEP control registers to show edge displacement for high-resolution deadband with ePWM in Up count mode due to the HRPWM control extension of the respective ePWM module.

\imageStyle{am263_hrpwm_deadband_sfo_fig1.PNG,width:80%}
\image html am263_hrpwm_deadband_sfo_fig1.PNG "Block diagram"

# External Connections
- EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC or AM263PX-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC 49 for epwm0_A
- Connect HSEC 51 for epwm0_B

## AM261X-LP
When using AM261x-LP
- Connect J5/J7 Pin 70 for epwm0_A
- Connect J6/J8 Pin 57 for epwm0_B

# Supported Combinations {#EXAMPLES_DRIVERS_HRPWM_DEADBAND_SFO_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

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
High Resolution Deadband 20.00 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 0

Calibration... Running...

High Resolution Deadband 20.14 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 7

Calibration... Running...

High Resolution Deadband 20.29 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 15

Calibration... Running...

High Resolution Deadband 20.44 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 23

Calibration... Running...

High Resolution Deadband 20.59 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 30

Calibration... Complete
SFO status=1 , MEP_ScaleFactor=46

High Resolution Deadband 20.74 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 38

Calibration... Running...

High Resolution Deadband 20.89 ns.
EPWM0 DBRED, DBFED = 8, DBREDHR, DBFEDHR = 46

\endcode

\imageStyle{am263_hrpwm_deadband_sfo_fig2.PNG,width:80%}
\image html am263_hrpwm_deadband_sfo_fig2.PNG "Expected output"

