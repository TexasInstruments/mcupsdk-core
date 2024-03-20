# QSPI Flash DMA LLD {#EXAMPLES_DRIVERS_QSPI_FLASH_DMA_LLD}

[TOC]

# Introduction

This example demonstrates the data transfer from flash connected to QSPI interface in DMA mode.
It uses the LLD Driver for DMA read functionality.

# Supported Combinations {#EXAMPLES_DRIVERS_QSPI_FLASH_DMA_LLD_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_dma_lld

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_dma_lld

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
[QSPI Flash Diagnostic Test] Starting ...
[QSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x1
[QSPI Flash Diagnostic Test] Flash Device ID       : 0x2018
[QSPI Flash Diagnostic Test] Executing Flash Erase on first block...
[QSPI Flash Diagnostic Test] Done !!!
[QSPI Flash Diagnostic Test] Performing Write-Read Test...
[QSPI Flash Diagnostic Test] Write-Read Test Passed!
All tests have passed!!
\endcode

