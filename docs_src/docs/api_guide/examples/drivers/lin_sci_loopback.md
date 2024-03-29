# LIN SCI Internal Loopback Interrupt {#EXAMPLES_DRIVERS_SCI_INTERNAL_LOOPBACK_INTERRUPT}

[TOC]

# Introduction

\cond SOC_AM263X || SOC_AM263PX
This example is a application which demonstrates the SCI message
communication.
Instance LIN1 is set to Transmit as well as Recieve.

The LIN1 instance initiates the transmission by sending SCI bytes.
It receives the same message.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SCI_INTERNAL_LOOPBACK_INTERRUPT_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/lin/lin_sci_loopback

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
[LIN] SCI Loopback mode, application started ...
All tests have passed!!
\endcode
