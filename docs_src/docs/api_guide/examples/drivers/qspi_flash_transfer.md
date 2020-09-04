# QSPI Flash Transfer {#EXAMPLES_DRIVERS_QSPI_FLASH_TRANSFER}

[TOC]

# Introduction

This example demonstrates the data transfer from flash connected to QSPI interface

# Supported Combinations {#EXAMPLES_DRIVERS_QSPI_FLASH_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_transfer

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER,
 Example folder | examples/drivers/qspi/qspi_flash_transfer

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[QSPI Flash Transfer Test] Starting ...
[QSPI Flash Transfer Test] Flash Manufacturer ID : 0x1
[QSPI Flash Transfer Test] Flash Device ID       : 0x2018
[QSPI Flash Transfer Test] Executing Flash Erase on first block...
[QSPI Flash Transfer Test] Done !!!
[QSPI Flash Transfer Test] Performing Write-Read Test...
[QSPI Flash Transfer Test] Write-Read Test Passed!
All tests have passed!!
\endcode

