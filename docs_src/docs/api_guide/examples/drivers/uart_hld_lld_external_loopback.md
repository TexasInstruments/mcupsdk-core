# UART HLD LLD External loopback{#EXAMPLES_DRIVERS_UART_HLD_LLD_EXTERNAL_LOOPBACK}

[TOC]

# Introduction

This example demonstrates the UART RX and TX operation from both
hld and lld driver APIs by looping the Tx & Rx externally in polling
mode of operation.

# External Connections :
## AM263x-CC E2 :
      Connect Jumper wires between the following HSEC pins
      - UART1_RXD(L3) J24 PIN-19  -----  UART4_TXD(C12)  J21 PIN-26
      - UART1_TXD(M3) J25 PIN-20  -----  UART4_RXD(D11)  J21 PIN-27
## AM263x-LP :
        Connect Jumper wires between the following pins
      - UART1_RXD(L3) J1/J3 PIN-3  -----  UART4_TXD(C9)  J6/J8 PIN-58
      - UART1_TXD(M3) J1/J3 PIN-4  -----  UART4_RXD(A10) J5/J7 PIN-47

# Supported Combinations {#EXAMPLES_DRIVERS_UART_HLD_LLD_EXTERNAL_LOOPBACK_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/uart/uart_hld_lld_external_loopback

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_UART_PAGE

# Sample Output

Shown below is a sample output when the application is run,
Please note that application prints in CCS console.

CCS Console:
\code
[UART] example started ...
This is uart test with HLD-LLD instances
All tests have passed!!
\endcode
