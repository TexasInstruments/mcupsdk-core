# CRC 16-Bit {#EXAMPLES_DRIVERS_CRC_16BIT}

[TOC]

# Introduction

This example demonstrates 16-bit Cyclic Redundancy Check (CRC) in full CPU mode for
Channel number 1. CRC signature is calculated on a frame that is stored in
memory and compared against pre-calculated CRC signature value.

# Supported Combinations {#EXAMPLES_DRIVERS_CRC_16BIT_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/crc/crc_16bit/

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
CRC 16-bit Test Started ...
CRC 16-bit Test Passed!!
All tests have passed!!
\endcode

