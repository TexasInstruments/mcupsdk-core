# I2C DMA Repeat Start {#EXAMPLES_DRIVERS_I2C_DMA_REPEAT_START}

[TOC]

# Introduction

This example demonstrates I2C communication with repeated start condition using DMA for efficient data transfer.

One of the cores controls an I2C Controller instance while the other core controls an I2C Target instance. The example uses two I2C instances - one configured as a controller which initiates the repeated start operation, and another configured as a target which responds to it.

The target core (R5FSS0-1) runs first to prepare for the I2C communication, then the controller core (R5FSS0-0) performs a write-then-read operation without releasing the I2C bus in between, using the repeated start condition. The target handles both receiving data from the controller and then transmitting data back to the controller after the repeated start.

The use of DMA significantly improves performance by freeing the CPU from handling byte-by-byte transfers, while the repeated start condition enables more efficient bus usage for combined write-read operations.

\cond SOC_AM263X 
\attention This example requires two I2C instances to be connected together.

SCL and SDA pins of I2C1 and I2C3 are available on the LP board.
I2C1 can be connected to I2C3 with following jumper Connections.

 - Connect Pin 9 of J1 (I2C1_SCL) to Pin 49 of J5 (I2C3_SCL).
 - Connect Pin 10 of J1 (I2C1_SDA) to Pin 50 of J5 (I2C3_SDA).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_DMA_REPEAT_START_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos (Controller)
 ^              | r5fss0-1 nortos (Target)
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_dma_repeat_start

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
  3. The controller will perform a write-read operation with repeated start, and the target will handle both receive and transmit
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_I2C_PAGE

# Sample Output

Shown below is a sample output when the application is run on the controller core:

\code

Cortex_R5_0: [I2C] Controller: requesting register 0x10 (8 bytes) ...
Cortex_R5_0: [I2C] Controller: register read verified
Cortex_R5_0: All tests have passed!!
Cortex_R5_0: [r5f0-1]     1.454649s : [I2C] Target: arming for repeated-start transfer ...
Cortex_R5_0: [r5f0-1]     1.584731s : [I2C] Target: transfer complete  responded to register 0x10
Cortex_R5_0: [r5f0-1]     1.584779s : All tests have passed!!

\endcode