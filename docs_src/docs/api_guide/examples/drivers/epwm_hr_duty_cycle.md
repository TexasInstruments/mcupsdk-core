# EPWM HR duty cycle {#EXAMPLES_DRIVERS_EPWM_HR_DUTY_CYCLE}

[TOC]

# Introduction

This example uses the ePWM to generate a PWM signal.

The example does the below
- Configures ePWM to generate the signal with a specified frequency and duty cycle.
- Waits for the specified time using ePWM ISR.

# External Connections
- EPWM9_A/B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect FSI header (on ControlCard) pin 8 for epwm9_A
- Connect FSI header (on ControlCard) pin 1 for epwm9_B

## AM263X-LP
When using AM263x-LP
- Connect boosterpack J6/J8 pin 75 for epwm9_A
- Connect boosterpack J6/J8 pin 76 for epwm9_B

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_HR_DUTY_CYCLE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_hr_duty_cycle/

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
EPWM Duty Cycle Test Started ...
App will wait for 60 seconds (using PWM period ISR) ...
EPWM Duty Cycle Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_epwm_hr_duty_cycle_output.PNG,width:80%}
 \image html am263_epwm_hr_duty_cycle_output.PNG "EPWM HR duty cycle waveform"
