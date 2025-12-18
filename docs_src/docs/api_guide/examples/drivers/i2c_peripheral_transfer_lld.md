# I2C Peripheral Transfer LLD {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_TRANSFER_LLD}

[TOC]

# Introduction

This example shows Master Peripheral Communication between Two I2C instances using LLD interface.

One of the core is in control of the I2C Master instance and the other core is in control of the I2C Peripheral instance.

The main core in in control of the Master I2C Instance.
The remote cores in in control of the Peripheral I2C Instance.

Three transactions are carried out in this example:
 1. Controller writes 4 bytes to Peripheral
 2. Controller writes 2 bytes to Peripheral
 3. Controller reads 2 bytes from Peripheral


\cond SOC_AM263X
\attention This Example requires two I2C instances to be connected together.

SCL and SDA pins of I2C1 and I2C3 are available on the LP board.
I2C1 can be connected to I2C3 with following jumper Connections.

 - Connect Pin 9 of J1 (I2C1_SCL) to Pin 49 of J5 (I2C3_SCL).
 - Connect Pin 10 of J1 (I2C1_SDA) to Pin 50 of J5 (I2C3_SDA).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_TRANSFER_LLD_COMBOS}

\cond SOC_AM263X 

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | 
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_peripheral_transfer_lld

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_I2C_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[I2C] I2C Controller Peripheral Transaction Started ... !!!
[I2C Peripheral] Transaction 1: Ready (receive 4)...
[I2C Controller] Transaction 1: Start (write 4)...
[I2C Controller] Transmitted Data 1 2 3 4 !!!
[I2C Peripheral] Received Data 1 2 3 4 !!!
[I2C Peripheral] Transaction 2: Ready (receive 2)...
[I2C Controller] Transaction 2: Start (write 2)...
[I2C Controller] Transmitted Data 1 2 !!!
[I2C Peripheral] Received Data 1 2 !!!
[I2C Peripheral] Transaction 3: Ready (send 2)...
[I2C Controller] Transaction 3: Start (read 2)...
[I2C Controller] Received Data 5 6 !!!
[I2C Peripheral] Transmitted Data 5 6 !!!

[I2C] All 3 transactions completed successfully!!!
All tests have passed!!

\endcode

