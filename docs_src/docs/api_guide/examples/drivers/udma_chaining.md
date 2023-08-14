# UDMA Chaining {#EXAMPLES_DRIVERS_UDMA_CHAINING}

[TOC]

# Introduction

UDMA chaining sample application performs a chain of block copy transfer
using channel global trigger.
Channel 0 completion triggers Channel 1 transfer: CH0 -> CH1.
\if SOC_AM65X
The application opens and configures two UDMA channel using SysConfig.
\else
The application opens and configures two BCDMA channel using SysConfig.
\endif

The first channel doesn't user a global trigger and each channel triggers
the next channel's global trigger through the channel's TR event register.

A channel's source buffer is previous channel's destination buffer. This
ensures that chaining trigger works in a synchronized manner when the
memory compare matches.

The transfer completion is based on last channel's DMA completion event.
Once the transfer it completes, it does cache operation for data coherency
and compares the source and destination buffers for any data mismatch.

# Supported Combinations {#EXAMPLES_DRIVERS_UDMA_CHAINING_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS, A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_chaining

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_chaining

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_chaining

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
[UDMA] Chaining application started ...
All tests have passed!!
\endcode
