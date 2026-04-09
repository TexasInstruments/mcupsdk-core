# I2C DMA Read Write {#EXAMPLES_DRIVERS_I2C_DMA_READ_WRITE}

[TOC]

# Introduction

This example demonstrates basic I2C read and write operations using DMA for efficient data transfer.

One of the cores controls an I2C Controller instance while the other core controls an I2C Target instance. The application uses two I2C instances - one configured as a controller (master) for transmitting data, and another configured as a target (slave) for receiving data.

The controller core (R5FSS0-0) transmits incremental data (0x00-0x07) to the target core (R5FSS0-1), which receives the data using DMA. The example validates that the data was transferred correctly.

The application uses the DMA to handle data transfers efficiently, offloading the CPU from having to manage individual byte transfers.

\cond SOC_AM263X || SOC_AM263PX
\attention This example requires two I2C instances to be connected together.

SCL and SDA pins of I2C1 and I2C3 are available on the LP board.
I2C1 can be connected to I2C3 with following jumper Connections.

 - Connect Pin 9 of J1 (I2C1_SCL) to Pin 49 of J5 (I2C3_SCL).
 - Connect Pin 10 of J1 (I2C1_SDA) to Pin 50 of J5 (I2C3_SDA).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_DMA_READ_WRITE_COMBOS}

\cond SOC_AM263X 

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos (Controller)
 ^              | r5fss0-1 nortos (Target)
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_dma_read_write

\endcond

# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- For multicore execution:
  1. First load and run the target core's executable on Core 1 (R5FSS0-1)
  2. Then load and run the controller core's executable on Core 0 (R5FSS0-0)
  3. The controller will transmit data and the target will receive and validate it
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_I2C_PAGE

# Sample Output

Shown below is a sample output when the application is run on the controller (master) core:

\code

Cortex_R5_0: [I2C] Controller: DMA write starting ...
Cortex_R5_0: [I2C] Controller: DMA write complete
Cortex_R5_0: All tests have passed!!
Cortex_R5_0: [r5f0-1]     1.107687s : [I2C] Target: DMA read starting ...
Cortex_R5_0: [r5f0-1]     3.236004s : [I2C] Target: DMA read complete  data verified
Cortex_R5_0: [r5f0-1]     3.236042s : All tests have passed!!

\endcode