# CRC FULL CPU {#EXAMPLES_DRIVERS_CRC_FULL_CPU}

[TOC]

# Introduction

This example demonstrates Cyclic Redundancy Check (CRC) in full CPU mode for
Channel number 1. CRC signature is calculated on a frame that is stored in
memory and compared against pre-calculated CRC signature value.

# Supported Combinations {#EXAMPLES_DRIVERS_CRC_FULL_CPU_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/crc/crc_full_cpu/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/crc/crc_full_cpu/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_CRC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
CRC Full CPU Test Started ...
Full CPU mode CRC Test Passed!!
All tests have passed!!
\endcode

