# I2C Slave Transfer {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_TRANSFER}

[TOC]

# Introduction

This example shows Master Slave Communication between Two I2C instances.

One of the core is in control of the I2C Master instance and the other core is in control of the I2C Slave instance.

The main core in in control of the Master I2C Instance.
The remote cores in in control of the Slave I2C Instance.

Two transactions are carried out in this example.

In the first transaction the Master Reads two bytes from slave in Blocking Mode.
In the second transaction the Master Writes two bytes to slave in Blocking Mode.


\cond SOC_AM263X || SOC_AM263PX
\attention This Example requires two I2C instances to be connected together.

SCL and SDA pins of I2C1 and I2C3 are available on the LP board.
I2C1 can be connected to I2C3 with following jumper Connections.

 - Connect Pin 9 of J1 (I2C1_SCL) to Pin 49 of J5 (I2C3_SCL).
 - Connect Pin 10 of J1 (I2C1_SDA) to Pin 50 of J5 (I2C3_SDA).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_TRANSFER_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_slave_transfer

\endcond

# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles whenbuilding the example.

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
[I2C SLAVE] I2C Master Slave Transaction Started ... !!!
[I2C MASTER] I2C Master Slave Transaction Started ... !!!
[I2C MASTER] Received Data 3 4 !!!
[I2C SLAVE] Transmitted Data 3 4 !!!
[I2C SLAVE] Received Data 1 2 !!!
[I2C MASTER] Transmitted Data 1 2 !!!
[I2C SLAVE] Transaction Complete !!!
[I2C MASTER] Transaction Complete !!!
All tests have passed!!
\endcode

