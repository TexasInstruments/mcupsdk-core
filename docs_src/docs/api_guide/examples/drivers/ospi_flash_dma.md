# OSPI Flash DMA {#EXAMPLES_DRIVERS_OSPI_FLASH_DMA}

[TOC]

# Introduction

This example demonstrates the OSPI low latency transfer using DMA. 16 bytes of known data are written to the flash. This data is then read back ten times by adjusting the TRPD for infinite TR reload. By this way the read can be done with really low latencies. The read back data is then compared with the known, sent data.

The average time for transfers is then printed out, and if all transfers were successful, the test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_FLASH_DMA_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_dma

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_dma

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
[OSPI] DMA low latency example started...
OSPI Write 16 bytes in 449758 ns
OSPI Read 16 bytes in 1888 ns
OSPI Read 16 bytes in 1578 ns
OSPI Read 16 bytes in 1508 ns
OSPI Read 16 bytes in 1498 ns
OSPI Read 16 bytes in 1498 ns
OSPI Read 16 bytes in 1498 ns
OSPI Read 16 bytes in 1498 ns
OSPI Read 16 bytes in 1502 ns
OSPI Read 16 bytes in 1498 ns
OSPI Read 16 bytes in 1498 ns
Average time for OSPI Read 16 bytes in 1546 ns
All tests have passed !!!
\endcode