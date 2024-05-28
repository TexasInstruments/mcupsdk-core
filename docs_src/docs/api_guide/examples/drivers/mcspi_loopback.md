# MCSPI Loopback {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK}

[TOC]

# Introduction

This example demonstrates the McSPI RX and TX operation configured
in blocking, interrupt mode of operation.

This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
and then receives the same in RX mode. Internal pad level loopback mode
is enabled to receive data.
To enable internal pad level loopback mode, D0 pin is configured to both
TX Enable as well as RX input pin in the SYSCFG.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | m4fss0-0 nortos(As am243x-lp has no MCU SPI, M4 core support excluded for am243x-lp)
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 freertos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback

\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCSPI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCSPI] Loopback example started ...
All tests have passed!!
\endcode

