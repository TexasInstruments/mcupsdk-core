# GPMC PSRAM {#EXAMPLES_DRIVERS_GPMC_PSRAM_IO}

[TOC]

# Introduction

This example demonstrate's basic read and write PSRAM using GPMC configured to operate in without DMA mode of operation. APIs from PSRAM driver are used to read and write to the psram. The underlying reads and writes are taken care by the PSRAM APIs.

The example writes known data to a particular offset in the psram and then reads it back. The read back data is then compared with the written known data.
When both the comparisons match, test result is passed otherwise failed.

An external GPMC PSRAM needs to be connected to the device.

# Supported Combinations {#EXAMPLES_DRIVERS_GPMC_PSRAM_IO_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpmc/gpmc_psram_io/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_GPMC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Starting GPMC application
All tests have passed
\endcode