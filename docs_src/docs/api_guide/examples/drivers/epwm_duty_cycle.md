# EPWM Duty Cycle {#EXAMPLES_DRIVERS_EPWM_DUTY_CYCLE}

[TOC]

# Introduction

This example generates a signal for a specified time and duty cycle using
ePWM module. The time and duty cycle can be configured by the user.

The example does the below
\if SOC_AM273X
- Configures ePWMA to generate a 1KHz signal with 25% duty cycle for 60 seconds.
- Output channel 0 is used in the example.
\else
- Configures ePWM0 to generate a 1KHz signal with 25% duty cycle for 60 seconds.
- Output channel A is used in the example.
\endif
- The parameters frequency, duty cycle and application run time are configurable by the user.
- Deadband submodule and chopper submodule are entirely bypassed in this example.
- ISR is used to keep track of keep track of elapsed time.
- Show usage of ePWM APIs
\cond SOC_AM64X
- For a53  this example uses poling method to generate pwm signal.
- To probe ePWM output you need to have IO breakout board..
- Short Pin 2 and 3 of J11 on IO break out board.
- Signal can be probed on Pin 7 of J6 on IO break out board.
\endcond

\cond SOC_AM243X

## AM243X-EVM
- To probe ePWM output you need to have IO breakout board..
- Short Pin 2 and 3 of J11 on IO break out board.
- Signal can be probed on Pin 7 of J6 on IO break out board.

## AM243X-LP
- Signal can be probed on Pin 1 of J2 header on am243x-lp board.
  You need to probe J4 Pin 40 which corresponds to Pin 1 of J2 header.

\endcond

\cond SOC_AM273X

## AM273X-EVM
- Signal can be probed on R81 resistor on am273x board.
\endcond

\cond SOC_AWR294X

## AWR294X-EVM
- Signal can be probed on R81 resistor on awr294x board.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DUTY_CYCLE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_duty_cycle/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_duty_cycle/

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_duty_cycle/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- To probe the ePWM output please refer setup details as mentioned above in Introduction section

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

