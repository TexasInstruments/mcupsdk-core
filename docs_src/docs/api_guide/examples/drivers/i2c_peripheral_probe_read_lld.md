# I2C Peripheral Probe Read LLD {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_PROBE_READ_LLD}

[TOC]

# Introduction

This example demonstrates Controller Peripheral Communication between Two I2C instances using LLD (Low-Level Driver) interface.

One I2C instance is configured as a Controller and the other as a Peripheral in the same core application.

The Controller I2C Instance manages master mode operations.
The Peripheral I2C Instance manages target/slave mode operations.

Seven transactions are carried out in this example, including I2C probe detection:

 1. Controller writes 4 bytes to Peripheral
 2. Controller reads 4 bytes from Peripheral
 3. I2C Probe (address-only detection)
 4. Controller writes 4 bytes again (different data)
 5. Controller reads 4 bytes again (different data)
 6. ReSTART condition (write 4 bytes, ReSTART, read 1 byte in single message)
 7. I2C Probe again (address-only detection)

The I2C Probe transactions demonstrate device discovery without data transfer, useful for detecting device presence on the bus.

\cond SOC_AM263X
\attention This Example requires two I2C instances to be connected together.

SCL and SDA pins of I2C1 (Controller) and I2C3 (Peripheral) are used in this example.
I2C1 can be connected to I2C3 with following jumper Connections on the LP board:

 - Connect Pin 9 of J1 (I2C1_SCL) to Pin 49 of J5 (I2C3_SCL).
 - Connect Pin 10 of J1 (I2C1_SDA) to Pin 50 of J5 (I2C3_SDA).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_PERIPHERAL_PROBE_READ_LLD_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_peripheral_probe_read_lld

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
[I2C] I2C Peripheral Probe & Read Test Started !!!
[TRANSACTION 1] Master Write: Controller -> Peripheral (4 bytes)
  TX (Controller)   : 01 02 03 04
  RX (Peripheral)   : 01 02 03 04
  [PASS]
[TRANSACTION 2] Master Read: Peripheral -> Controller (4 bytes)
  TX (Peripheral)   : 05 06 07 08
  RX (Controller)   : 05 06 07 08
  [PASS]
[TRANSACTION 3] I2C Probe: address-only detection
  Device ACK'd address 0x2C – probe successful
  [PASS]
[TRANSACTION 4] Master Write (again): Controller -> Peripheral (4 bytes)
  TX (Controller)   : 11 12 13 14
  RX (Peripheral)   : 11 12 13 14
  [PASS]
[TRANSACTION 5] Master Read (again): Peripheral -> Controller (4 bytes)
  TX (Peripheral)   : 15 16 17 18
  RX (Controller)   : 15 16 17 18
  [PASS]
[TRANSACTION 6] ReSTART: Controller writes 4, ReSTART, reads 1
  Phase 1 TX (Controller)   : 21 22 23 24
  Phase 1 RX (Peripheral)   : 21 22 23 24
  Phase 2 TX (Peripheral)   : 25
  Phase 2 RX (Controller)   : 25
  [PASS]
[TRANSACTION 7] I2C Probe (again): address-only detection
  Device ACK'd address 0x2C – probe successful
  [PASS]

[I2C] All 7 transactions (Probe & Read Test) completed successfully!!!
All tests have passed!!

\endcode

