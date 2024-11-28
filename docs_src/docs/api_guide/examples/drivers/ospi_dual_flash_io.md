# OSPI Dual Flash IO {#EXAMPLES_DRIVERS_OSPI_DUAL_FLASH_IO}

[TOC]

# Introduction

This example demonstrates basic read write to both of the OSPI flash instances configured in polled mode of operation. APIs from flash driver are used to read and write to both the flashes. The underlying OSPI reads and writes are taken care by the flash APIs.

The example writes known data to a particular offset in the flashes and then reads it back. The read back data is then compared with the written known data.

When the comparisons match on both the flash read's, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_DUAL_FLASH_IO_COMBOS}

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_dual_flash_io

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_OSPI_PAGE

# Sample Output

\code
All tests have passed!!
\endcode