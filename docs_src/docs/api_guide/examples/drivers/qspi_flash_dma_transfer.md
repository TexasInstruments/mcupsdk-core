# QSPI Flash Dma Transfer {#EXAMPLES_DRIVERS_QSPI_FLASH_DMA_TRANSFER}

[TOC]

# Introduction

This example demonstrates the data transfer from flash connected to QSPI interface in DMA mode

# Supported Combinations {#EXAMPLES_DRIVERS_QSPI_FLASH_DMA_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_dma_transfer

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_dma_transfer

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
[QSPI Flash DMA Transfer Test] Starting ...
[QSPI Flash DMA Transfer Test] Executing Flash Erase on first block...
[QSPI Flash DMA Transfer Test] Performing Write-Read Test...
All tests have passed!!
\endcode

