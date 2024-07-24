# GPTIMER Overflow Callback {#EXAMPLES_DRIVERS_GP_TIMER_OVERFLOW_CALLBACK}

[TOC]

# Introduction

This example demonstrates the timer's functionality in free run mode. In the Example interrupt is enabled. and a overflow user callback is registered. In the callback a semaphore is posted.

The application sets the counter value to 0xFF000000 and starts the timer and pends semaphore.
On overflow event the callback is called and semaphore is posted.

Application stops the timer after semaphore post.

The counter source Clock is set to MCU_HFOSC0 giving it a 25 MHz tick with counter Presacler disabled.

# Supported Combinations {#EXAMPLES_DRIVERS_GP_TIMER_OVERFLOW_CALLBACK_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_overflow_callback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_overflow_callback

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
GP Timer Overflow Callback Test Started ...
Overflow Interrupt triggered !!
All tests have passed!!
\endcode
