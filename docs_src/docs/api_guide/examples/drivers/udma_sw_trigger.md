# UDMA SW Trigger {#EXAMPLES_DRIVERS_UDMA_SW_TRIGGER}

[TOC]

# Introduction

UDMA SW trigger sample application performs 3D transfer using SW trigger method as below
Loop N times (icnt2)
- SW trigger CH 0 -> Triggers MSMC to Intermediate buffer
- Wait for CH 0 icnt0 x icnt1 to complete
- SW trigger Channel 1 -> Triggers Intermediate buffer to MSMC
- Wait for CH 1 icnt0 x icnt1 to complete

Each loop transfers M (icnt0 x icnt1) bytes of data.
MSMC size is M x N and intermediate buffer size is just M bytes.
Intermediate buffer memory set to wrap around after M bytes of transfer.

Where,
- M is icnt0 x icnt1 (UDMA_TEST_1D_SIZE x UDMA_TEST_2D_SIZE)
- N is icnt2 = UDMA_TEST_3D_SIZE

Once the transfer it completes, it does cache operation for data coherency
and compares the source and destination buffers for any data mismatch.

# Supported Combinations {#EXAMPLES_DRIVERS_UDMA_SW_TRIGGER_COMBOS}

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
 Example folder | examples/drivers/udma/udma_sw_trigger

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_sw_trigger

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_sw_trigger

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
[UDMA] SW Trigger application started ...
All tests have passed!!
\endcode
