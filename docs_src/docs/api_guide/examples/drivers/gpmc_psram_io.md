# GPMC PSRAM {#EXAMPLES_DRIVERS_GPMC_PSRAM_IO}

[TOC]

# Introduction

This example demonstrates basic read and write to PSRAM using GPMC. The GPMC is configured to operate without DMA. APIs from RAM driver are used to read and write to the psram. The underlying reads and writes are taken care by the PSRAM APIs.

The example writes known data to a particular offset in the psram and then reads it back. The read back data is then compared with the written known data.
When both the data match, throughput is calculated & test result is passed otherwise failed.

An external PSRAM needs to be connected to the device via GPMC interface.

\cond SOC_AM64X || SOC_AM243X
\note
    - **TI GPMC Daughter Card:** A 16-bit pSRAM of 64Mb capacity is present on board.
    - For 16-bit devices, the address lines SoC_GPMC A1 should be conected to Device A0 & so on. On Daughter Card, SoC_GPMC A0 is connected to Device A0 line.
    - Due to this only half the capacity is accessible (32Mb of 64Mb) & addresses are accessed in alternate manner (address 0, 2, 4 ..) with each address corresponding to 16 bits of data.
\endcond
# Supported Combinations {#EXAMPLES_DRIVERS_GPMC_PSRAM_IO_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpmc/gpmc_psram_io/

\endcond
\cond SOC_AM64X || SOC_AM243X

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

\cond SOC_AM263X
\code
Starting GPMC application
All tests have passed
\endcode
\endcond
\cond SOC_AM64X
\code
Write Speed: 40.972553 Mbps
Read Speed: 44.413368 Mbps
All tests have passed!!
\endcode
\endcond
\cond SOC_AM243X
\code
Write Speed: 81.980049 Mbps
Read Speed: 59.017868 Mbps
All tests have passed!!
\endcode
\endcond
