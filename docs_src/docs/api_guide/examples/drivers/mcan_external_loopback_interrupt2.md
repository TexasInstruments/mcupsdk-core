# MCAN External Loopback Interrupt {#EXAMPLES_DRIVERS_MCAN_EXTERNAL_LOOPBACK_INTERRUPT2}

[TOC]

# Introduction

\cond SOC_AM64X
This example is a multi core application which demonstrates the CAN message
communication between the MCAN instances.
Instance MCAN1 is designated as 'TX'
and the other instance MCAN0 is designated as 'RX'.

The MCAN1 instance initiates the transmission by sending a message.
The MCAN0 instance receives the same message.
Message is transmitted with the following configuration.
\endcond

- CAN FD Message Format.
- Message ID Type is Standard, Message Id is defined as 0xC0.
- MCAN is configured in Interrupt Mode.
- MCAN Interrupt Line Number 0.
- Arbitration Bit Rate 1Mbps.
- Data Bit Rate 5Mbps.
- Buffer mode is used for both TX/RX to send/receive message in message RAM.

This example runs for 10 iterations and in each
iteration the received message id and the data is compared with the transmitted
one. After 10 iterations the example is completed.
This is a example CAN communication, user can configure different message
formats as needed for their applications.

These 2 MCAN instances are connected externally.
Below are the connection details.
\cond SOC_AM64X
- All pins are on the base board.
- Pin number 1 starts from the triangular arrow mark.
 MCAN0                   | MCAN1
 ------------------------|-----------------
 MCAN0_H(Pin 1  of J31)  | MCAN1_H(Pin 1  of J32)
 MCAN0_L(Pin 3  of J31)  | MCAN1_L(Pin 3  of J32)
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_MCAN_EXTERNAL_LOOPBACK_INTERRUPT2_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_external_loopback_interrupt

\endcond

\cond SOC_AM64X
\note MCAN transceiver is enabled in RX application via mcan_enableTransceiver() function.
To enable transceiver, I2C1 should be configured in SYSCFG.
\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- This is a multi-core example. Hence the executables should be loaded and run for all the above mentioned cores
- The application has a sync mechanism at the start which waits for all cores to start before doing the test. Hence the cores can be loaded and run in any sequence.

# See Also

\ref DRIVERS_MCAN_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCAN] RX Application, Interrupt mode started ...
[a530-0]     0.001071s : [MCAN] TX Application, Interrupt mode started ...
All tests have passed!!
[a530-0]     0.014002s : [MCAN]TX test passed!!
\endcode
