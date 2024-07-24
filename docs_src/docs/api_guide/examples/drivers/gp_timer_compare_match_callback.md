# GPTIMER Compare match Callback {#EXAMPLES_DRIVERS_GP_TIMER_COMPARE_MATCH_CALLBACK}

[TOC]

# Introduction

This example demonstrates the timer's functionality in compare match mode. In the Example interrupt is enabled and a compare match user callback is registered. In the callback the counter is reset to zero and a match count variable is incremented.
The application waits until the match count variable updates and on update prints its value.
Application stops the timer and exits when match count reaches 10.

The counter source Clock is set to MCU_HFOSC0 giving it a 25 MHz tick with the counter Presacler disabled.
Compare value is set to 12500000. Therefore the callback is called approximately twice every second.

# Supported Combinations {#EXAMPLES_DRIVERS_GP_TIMER_COMPARE_MATCH_CALLBACK_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_compare_match_callback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gp_timer/gp_timer_compare_match_callback

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
GP Timer Compare Match Test Started ...
Compare Match Flag set, matchCount: 1
Compare Match Flag set, matchCount: 2
Compare Match Flag set, matchCount: 3
Compare Match Flag set, matchCount: 4
Compare Match Flag set, matchCount: 5
Compare Match Flag set, matchCount: 6
Compare Match Flag set, matchCount: 7
Compare Match Flag set, matchCount: 8
Compare Match Flag set, matchCount: 9
Compare Match Flag set, matchCount: 10
All tests have passed!!
\endcode
