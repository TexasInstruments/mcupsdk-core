# UART Echo Low Latency Interrupt {#EXAMPLES_DRIVERS_UART_ECHO_LOW_LATENCY_INTERRUPT}

[TOC]

# Introduction

This example demonstrate the UART low latency API and user managed interrupt
service routine.
This example receives 8 characters and echos back the same.
The application ends when the user types 8 characters.

Initially the application sets a buffer to receive data and enables the
RX interrupt. When RX ISR is triggered the ISR reads the data from UART
FIFO and write to the RX buffer and sets the RX buffer count.

In the main context, the application checks if RX buffer has any data
and if so reads the data from it and copy it to TX buffer and initiate the
UART TX (echo).

# Supported Combinations {#EXAMPLES_DRIVERS_UART_ECHO_LOW_LATENCY_INTERRUPT_COMBOS}

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
 Example folder | examples/drivers/uart/uart_echo_low_latency_interrupt

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_interrupt

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_interrupt

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_echo_low_latency_interrupt

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
[UART] Echo Low Latency interrupt mode example started ...
All tests have passed!!
\endcode

UART Console:
\code
This is uart low latency test in interrupt mode, Receives 8 characters then echo's back and exits..
12345678
All tests have passed!!
\endcode
