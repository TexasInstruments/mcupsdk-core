# GPMC NAND Flash {#EXAMPLES_DRIVERS_GPMC_FLASH_NAND_IO}

[TOC]

# Introduction

This example demonstrate's basic read write erase to the Parallel NAND flash using GPMC configured to operate in DMA mode of operation. APIs from flash driver are used to read, write and erase to the flash. The underlying reads and writes are taken care by the flash APIs.

The example writes known data to a particular offset in the flash and then reads it back. The read back data is then compared with the written known data.
By deafult ELM is enabled in the example. When both the comparisons match, test result is passed otherwise failed.

An external GPMC NAND flash needs to be connected to the device.

# Supported Combinations {#EXAMPLES_DRIVERS_GPMC_FLASH_IO_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpmc/gpmc_flash_io/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpmc/gpmc_flash_io/

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
All tests have passed
\endcode
