# UART Echo Low Latency Polling {#EXAMPLES_DRIVERS_UART_ECHO_LOW_LATENCY_POLLING}

[TOC]

# Introduction

This example demonstrate the UART low latency API in polling mode.
This example receives 8 characters and echos back the same.
The application ends when the user types 8 characters.
Initially the application sets a buffer to receive data.

In the main context, the application checks any data from the UART
FIFO and if so, write to the RX buffer and sets the RX buffer count.
Application then copies the data to TX buffer and initiate the UART TX (echo).

# Supported Combinations {#EXAMPLES_DRIVERS_UART_ECHO_LOW_LATENCY_POLLING_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_polling

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_polling

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_polling

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_polling

\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Please select the UART port used for console as mentioned in \ref CCS_UART_TERMINAL

# See Also

\ref DRIVERS_UART_PAGE

# Sample Output

Shown below is a sample output when the application is run,
Please note that application prints in both CCS and UART console.
In UART console you need to type 8 characters.

CCS Console:
\code
[UART] Echo Low Latency polling mode example started ...
All tests have passed!!
\endcode

UART Console:
\code
This is uart low latency test in polling mode, Receives 8 characters then echo's back and exits..
12345678
All tests have passed!!
\endcode
