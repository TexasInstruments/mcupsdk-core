# EDMA MultiMem Transfer {#EXAMPLES_DRIVERS_EDMA_MULTIMEM_TRANSFER}

[TOC]

# Introduction

This example demonstrates the data transfer using EDMA between different memories in the SOC. It also prints out the time taken for each transfer.


# Supported Combinations {#EXAMPLES_DRIVERS_EDMA_MULTIMEM_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_multimem_transfer

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_multimem_transfer/

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
[Cortex_R5_0] [EDMA] Interrupt Transfer Test Started...
[EDMA MULTIMEM TRANSFER] OCRAM to OCRAM Total transfer time for 1KB = 52 usecs
[EDMA MULTIMEM TRANSFER] TCMA to TCMA Total transfer time for 1KB = 47 usecs
[EDMA MULTIMEM TRANSFER] TCMB to TCMB Total transfer time for 1KB = 48 usecs
[EDMA MULTIMEM TRANSFER] OCRAM to TCMA Total transfer time for 1KB = 47 usecs
[EDMA MULTIMEM TRANSFER] TCMA to OCRAM Total transfer time for 1KB = 49 usecs
[EDMA] Interrupt Transfer Test Completed!!
All tests have passed!!
\endcode

