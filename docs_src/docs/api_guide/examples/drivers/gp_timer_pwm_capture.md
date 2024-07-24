# GPTIMER PWM Capture {#EXAMPLES_DRIVERS_GP_TIMER_PWM_CAPTURE}

[TOC]

# Introduction

This example demonstrates the timer's functionality in PWM Generation mode and input capture mode.
In this Example one Timer is configured in PWM_GEN mode and the other one is configured in INPUT_CAPTURE mode. The first timer is configured to generate a 1 KHz Square wave.

The other timer is configured to capture two consecutive edges and call the callback on second capture. In the callback the capture values are read, the difference is stored and a semaphore is posted.

Application starts the capture timer first and then starts the PWM generating timer and pends semaphore to be posted on capture. On post application stops both counters and exits.

The counter source Clock in both cases is set to MCU_HFOSC0 giving it a 25 MHz tick with counter Presacler disabled.

\cond SOC_AM243X
 - Connect Pin 74 of J8 (MMC1_SDCD) to Pin 73 of J8 (MMC1_SDWP).
 - Example is not supported for EVM Board.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_GP_TIMER_PWM_CAPTURE_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_pwm_capture

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_GPTIMER_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
GP Timer PWM Capture Test Started ...
Ticks between Rising and falling edge : 12500
All tests have passed!!
\endcode
