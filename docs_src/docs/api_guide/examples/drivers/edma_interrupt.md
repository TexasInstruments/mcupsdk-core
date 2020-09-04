# EDMA Interrupt Transfer {#EXAMPLES_DRIVERS_EDMA_INTERRUPT_TRANSFER}

[TOC]

# Introduction

This example demonstrates the data transfer using EDMA in interrupt mode

# Supported Combinations {#EXAMPLES_DRIVERS_EDMA_INTERRUPT_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_interrupt_transfer

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_interrupt_transfer/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
\if SOC_AM263X
- Launch a CCS debug session, follow the steps for running multi core applications in \ref PREREQUISITES and run the executables, see \ref CCS_LAUNCH_PAGE
\else
- Launch a CCS debug session and run the executables, see \ref CCS_LAUNCH_PAGE
\endif
- This is a multi-core example. Hence the executables should be loaded and run for all the above mentioned cores

# Sample Output

Shown below is a sample output when the application is run,

\code
[EDMA] Interrupt Transfer Test Started...
[EDMA] Interrupt Transfer Test Completed!!
All tests have passed!!
\endcode

