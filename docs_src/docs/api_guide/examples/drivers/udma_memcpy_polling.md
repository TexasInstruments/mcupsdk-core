# UDMA Memcpy Polling {#EXAMPLES_DRIVERS_UDMA_MEMCPY_POLLING}

[TOC]

# Introduction

This example performs UDMA block copy transfer using Type 15 Transfer Record (TR15)
using Transfer Record Packet Descriptor (TRPD) in polling mode.
\if SOC_AM65X
The application opens and configures a UDMA channel using SysConfig.
\else
The application opens and configures a BCDMA channel using SysConfig.
\endif
Then the application prepares a TRPD for a 1D transfer from source to
destination buffer, submits the request to DMA, waits for the DMA to complete
by polling the channels's completion ring accelerator (RA).

Once the transfer it completes, it does cache operation for data coherency
and compares the source and destination buffers for any data mismatch.

# Supported Combinations {#EXAMPLES_DRIVERS_UDMA_MEMCPY_POLLING_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS, A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_memcpy_polling

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_memcpy_polling

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_memcpy_polling

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_UDMA_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[UDMA] Memcpy application started ...
All tests have passed!!
\endcode
