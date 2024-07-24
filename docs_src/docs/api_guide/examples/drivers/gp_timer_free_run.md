# GPTIMER Free Run {#EXAMPLES_DRIVERS_GP_TIMER_FREE_RUN}

[TOC]

# Introduction

This example demonstrates the timer's functionality in free run mode. In the Example interrupt is disabled.
The application waits approximately 1 second and calculates the difference in ticks before and after the wait. This gives an approximate timer frequency.
Application stops the timer after 5 such approximations.

The counter source Clock is set to MCU_HFOSC0 giving it a 25 MHz tick with counter Presacler disabled.
Therefore the approximated frequency is close to 25000000.

# Supported Combinations {#EXAMPLES_DRIVERS_GP_TIMER_FREE_RUN_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_free_run

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_free_run

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
GP Timer Free Run Test Started ...
Approximate Timer Frequency: 24992259
Approximate Timer Frequency: 24976866
Approximate Timer Frequency: 24981528
Approximate Timer Frequency: 24983136
Approximate Timer Frequency: 24985422
All tests have passed!!
\endcode
