# MCAN External Read Write {#EXAMPLES_DRIVERS_MCAN_EXTERNAL_READ_WRITE}

[TOC]

# Introduction

This example demonstrates (tested on) the CAN message communication to external PC via PCAN-USB (from PEAK Systems : IPEH-002021) in with the following configuration.

- CAN FD Message Format.
- Message ID Type is Standard, Msg Id 0xC0.
- MCAN is configured in Interrupt Mode.
- MCAN Interrupt Line Number 0.
- Arbitration Bit Rate 1Mbps.
- Data Bit Rate 5Mbps.
- Buffer mode is used for Tx and RX to store message in message RAM.

Instance MCAN1 is set as a Commander in Transmit Mode. Message is transmitted and received back from external PC via PCAN-USB. Once message is transmitted the example will wait for recieving the messages from external PC. Have to manually transmit ten messages from external PC for test to finish. When the received message id and the data matches with the transmitted one, then the example is completed.

# Supported Combinations {#EXAMPLES_DRIVERS_MCAN_EXTERNAL_READ_WRITE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcan/mcan_external_read_write

\endcond

# Steps to Run the Example

- **Hardware Conectivity**, connect the PCAN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.

\imageStyle{am263x_mcan_external_hw_connect.PNG,width:40%}
\image html am263x_mcan_external_hw_connect.PNG MCAN Hardware Connectivity with PCAN USB.

- **Software Setup**, Download and Install the PCAN-View from https://www.peak-system.com/PCAN-View.242.0.html?&L=1
- Click on CAN in the menu bar and conncet to PCAN-Usb. Set Mode as ISO CAN FD and sampling point in Nominal Bit Rate as 85 percent and Sampling Point in Data Bit Rate as 87.5 percent. Rest as default.

- When data is recieved from MCAN instance, it will be visible as shown in the image below in PCAN-View.
\imageStyle{am263x_pcan_receive_status.PNG,width:50%}
\image html am263x_pcan_receive_status.PNG Data received as shown in PCAN-View.

- After successful reception of data, the data have to be transmitted ten times from PCAN-View for test to end.

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
[MCAN] External read - write test, application started ...
After transmitting messages it will wait to recieve ten messages for test to pass ...
All tests have passed!!
\endcode

