# MIBSPI Loopback {#EXAMPLES_DRIVERS_MIBSPI_LOOPBACK}

[TOC]

# Introduction

This example demonstrates the MibSPI RX and TX operation configured
in blocking, interrupt mode of operation.

This example sends a known data in the TX mode of length 
APP_MIBSPI_MSGSIZE and then receives the same in RX mode. 
Digital loopback mode is enabled to receive data.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_MIBSPI_LOOPBACK_COMBOS}

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mibspi/mibspi_loopback
 
\endcond
 
\cond SOC_AWR294X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mibspi/mibspi_loopback

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MIBSPI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MIBSPI] Digital Loopback example started ...
All tests have passed!!
\endcode

