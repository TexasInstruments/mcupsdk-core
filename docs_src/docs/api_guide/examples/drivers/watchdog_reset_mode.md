# Watchdog reset mode {#EXAMPLES_DRIVERS_WATCHDOG_RESET_MODE}

[TOC]

# Introduction

This example uses the WDT module in reset mode to trigger SOC warm reset.
Servicing the WDT for few iterations before triggering the warm reset.
It will reset the device once it reaches the expiry time set by user.
Note that this example won't work in the debug mode as the user must not service the watchdog while in debug mode.
For it to work, we can flash the image to get the expected result.

# Supported Combinations {#EXAMPLES_DRIVERS_WATCHDOG_RESET_MODE_COMBOS}
\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_reset/

\endcond

# Supported Combinations
\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_reset/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Please select the UART port used for console as mentioned in \ref CCS_UART_TERMINAL

# See Also

\ref DRIVERS_WATCHDOG_PAGE

# Sample Output

Shown below is a sample output when the application is run,
Please note that application prints in both CCS and UART console.

\note Console prints will be stopped once WDT triggers warm reset.

CCS Console:
\code
[MAIN_Cortex_R5_0_0] Watchdog reset Mode Test Started ...
Servicing WDT for few iterations
Watchdog triggers warm reset in 3000 (ms)
All tests have passed!!
Prints will be stopped after reset
Prints will be stopped after reset
Prints will be stopped after reset
'' ''
'' ''
'' ''
Prints will be stopped after reset
\endcode

UART Console:
\code
Watchdog reset Mode Test Started ...
Servicing WDT for few iterations
Watchdog triggers warm reset in 3000 (ms)
All tests have passed!!
Prints will be stopped after reset
Prints will be stopped after reset
Prints will be stopped after reset
'' ''
'' ''
'' ''
Prints will be stopped after reset
\endcode

