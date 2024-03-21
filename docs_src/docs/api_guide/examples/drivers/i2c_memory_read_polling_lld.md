# I2C Memory Read Polling LLD {#EXAMPLES_DRIVERS_I2C_MEMORY_READ_POLLING_LLD}

[TOC]

# Introduction

This example demonstrates writing data to & reading data from the EEPROM
connected to the I2C controller.
Application writes 10 Bytes to the EEPROM in interrupt mode and then reads
back the same 10 Bytes.

\cond SOC_AM64X
\attention For AM64xSK, Board ID EEPROM address is 0x51u.
\endcond

The application makes use of the I2C LLD driver and it's memory read/write
polling APIs.

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_MEMORY_READ_POLLING_LLD_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_memory_read_polling_lld

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_memory_read_polling_lld

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_memory_read_polling_lld

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_memory_read_polling_lld

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_I2C_LLD_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond SOC_AM263X || SOC_AM263PX

\code
[I2C] LLD Memory Read test Started ... !!!
[I2C] Application will write 10 consecutive bytes in polling Mode to EEPROM memory address 0x400 ...
[I2C] Application will Read 10 consecutive bytes in polling Mode from EEPROM memory address 0x400 ...
[I2C] Data at address 0x400 : 0x0
[I2C] Data at address 0x401 : 0x1
[I2C] Data at address 0x402 : 0x2
[I2C] Data at address 0x403 : 0x3
[I2C] Data at address 0x404 : 0x4
[I2C] Data at address 0x405 : 0x5
[I2C] Data at address 0x406 : 0x6
[I2C] Data at address 0x407 : 0x7
[I2C] Data at address 0x408 : 0x8
[I2C] Data at address 0x409 : 0x9
All tests have passed!!
\endcode

\endcond

\cond SOC_AM273X || SOC_AWR294X

\code
[I2C] LLD Memory Read test Started ... !!!
[I2C] Application will write 10 consecutive bytes in polling Mode to EEPROM memory address 0x30 ...
[I2C] Application will Read 10 consecutive bytes in polling Mode from EEPROM memory address 0x30 ...
[I2C] Data at address 0x30 : 0x0
[I2C] Data at address 0x31 : 0x1
[I2C] Data at address 0x32 : 0x2
[I2C] Data at address 0x33 : 0x3
[I2C] Data at address 0x34 : 0x4
[I2C] Data at address 0x35 : 0x5
[I2C] Data at address 0x36 : 0x6
[I2C] Data at address 0x37 : 0x7
[I2C] Data at address 0x38 : 0x8
[I2C] Data at address 0x39 : 0x9
All tests have passed!!
\endcode

\endcond
