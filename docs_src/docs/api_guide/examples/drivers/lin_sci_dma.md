# LIN SCI DMA Loopback {#EXAMPLES_DRIVERS_SCI_DMA_LOOPBACK}

[TOC]

# Introduction

\cond SOC_AM263X
This example is a application which demonstrates the SCI message
communication via DMA.
Instance LIN1 is set to Transmit as well as Recieve.

DMAMUX and EDMA to be configured to send the interrupts to LIN peripheral.

The LIN1 instance initiates the transmission by setting the LIN_SET_TX_DMA.
It receives the same message via RX DMA if LIN_SET_RX_DMA is set.

We are sending 128 bytes of data with 8 bytes in a single transfer.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SCI_DMA_LOOPBACK_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/lin/lin_sci_dma

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_LIN_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[LIN] SCI DMA mode, application started ...
[LIN] Initialization of DMA started ...
All tests have passed!!
\endcode
