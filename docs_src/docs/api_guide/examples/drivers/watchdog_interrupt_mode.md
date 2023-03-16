# Watchdog interrupt mode {#EXAMPLES_DRIVERS_WATCHDOG_INTERRUPT_MODE}

[TOC]

# Introduction

This example uses the WDT module in non reset mode to generate NMI Interrupt.
- The Watchdog interrupt is configured as a non-maskable interrupt and the user-defined callback function is registered.
\cond SOC_AM273X
- ESM module is configured with ESM Group 2 number and ESM NMI number to generate a non-maskable interrupt to the CPU.
\endcond
\cond SOC_AM263X
- ESM module is configured with ESM to generate a non-maskable interrupt to the CPU.
\endcond
- The callback function in the application handles the watchdog interrupt.

# Supported Combinations {#EXAMPLES_DRIVERS_WATCHDOG_INTERRUPT_MODE_COMBOS}
\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_interrupt/

\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_interrupt/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_interrupt/

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/watchdog/watchdog_interrupt/

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

CCS Console:
\code
[Cortex_R5_0] Watchdog interrupt Mode Test Started ...
Watchdog Driver NMI received
All tests have passed!!
\endcode

UART Console:
\code
Watchdog interrupt Mode Test Started ...
Watchdog Driver NMI received
All tests have passed!!
\endcode

