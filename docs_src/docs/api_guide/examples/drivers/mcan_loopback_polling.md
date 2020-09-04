# MCAN Loopback Polling {#EXAMPLES_DRIVERS_MCAN_LOOPBACK_POLLING}

[TOC]

# Introduction

This example demonstrates the CAN message transmission and reception in
digital loop back mode with the following configuration.

- Classic CAN Message Format.
- Message ID Type is Extended, Msg Id 0xD0, 0xD1, 0xD2, 0xD3, 0xD4.
- MCAN is configured in Polling Mode.
- Arbitration Bit Rate 1Mbps.
- Data Bit Rate 5Mbps.
- FIFO mode is used for Tx and RX to store message in message RAM.

5 Messages are transmitted and received back internally using internal loopback
mode. When all the 5 messages received with the id and the data matches
with the transmitted one, then the example is completed.

# Supported Combinations {#EXAMPLES_DRIVERS_MCAN_LOOPBACK_POLLING_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_loopback_polling

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_loopback_polling

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_loopback_polling

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_loopback_polling

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 freertos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_loopback_polling

\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCAN_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCAN] Loopback Polling mode, application started ...
All tests have passed!!
\endcode

