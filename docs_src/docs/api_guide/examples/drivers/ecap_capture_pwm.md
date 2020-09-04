# ECAP Capture Pwm {#EXAMPLES_DRIVERS_ECAP_CAPTURE_PWM}

[TOC]

# Introduction

This example uses the ECAP in capture mode to capture PWM.

The example does the below
- Configures ECAP in Capture mode and captures the epwm output.
- It captures the time between the rising and falling edge of epwm output.

```mermaid
  ___________________          _____________             __________________
  |                 |          |           |             |                |
  |      EPWM       |---GPIO---| I/P XBAR  |---I/P MUX---|     ECAP       |---INT XBAR
  |_________________|          |___________|             |________________|

```

# External Connections

## AM263X-CC
No external connection is required.

## AM263X-LP
No external connection is required.

# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_CAPTURE_PWM_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_capture_pwm/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_ECAP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ECAP Capture Pwm Test Started ...
Interrupt No.: 10 and Pass Count: 10
ECAP Capture Pwm Passed!!
All tests have passed!!
\endcode
