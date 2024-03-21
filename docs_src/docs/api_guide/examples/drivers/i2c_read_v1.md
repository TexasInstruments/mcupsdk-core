# I2C Read {#EXAMPLES_DRIVERS_I2C_READ_V1}

[TOC]

# Introduction

This example demonstrates reading data from I2C based EEPROM devices present in the board.
Application reads 20 samples from the EEPROM and exits.

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_READ_V1_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

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
[I2C] Read data ... !!!
[I2C] Sample 0: 49
[I2C] Sample 1: 48
[I2C] Sample 2: 48
[I2C] Sample 3: 49
[I2C] Sample 4: 65
[I2C] Sample 5: 0
[I2C] Sample 6: 0
[I2C] Sample 7: 0
[I2C] Sample 8: 48
[I2C] Sample 9: 49
[I2C] Sample 10: 48
[I2C] Sample 11: 49
[I2C] Sample 12: 51
[I2C] Sample 13: 50
[I2C] Sample 14: 50
[I2C] Sample 15: 51
[I2C] Sample 16: 0
[I2C] Sample 17: 0
[I2C] Sample 18: 0
[I2C] Sample 19: 0
[I2C] Read data ... DONE !!!
All tests have passed!!
\endcode
