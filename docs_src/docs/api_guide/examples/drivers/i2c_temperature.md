# I2C Temperature Read {#EXAMPLES_DRIVERS_I2C_TEMPERATURE}

[TOC]

# Introduction

This example demonstrates probing of sensor via I2C and read data from it.
The application reads data from the temperature sensor in the EVM.
Application reads 20 samples from the sensor and exits.

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_TEMPERATURE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature

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
[I2C] Temperature sensor found at device address 0x48
[I2C] Sample 0: 39.000000 (celcius)
[I2C] Sample 1: 39.000000 (celcius)
[I2C] Sample 2: 39.000000 (celcius)
[I2C] Sample 3: 39.000000 (celcius)
[I2C] Sample 4: 39.000000 (celcius)
[I2C] Sample 5: 39.000000 (celcius)
[I2C] Sample 6: 39.000000 (celcius)
[I2C] Sample 7: 39.000000 (celcius)
[I2C] Sample 8: 39.000000 (celcius)
[I2C] Sample 9: 39.000000 (celcius)
[I2C] Sample 10: 39.000000 (celcius)
[I2C] Sample 11: 39.000000 (celcius)
[I2C] Sample 12: 39.000000 (celcius)
[I2C] Sample 13: 39.000000 (celcius)
[I2C] Sample 14: 39.000000 (celcius)
[I2C] Sample 15: 39.000000 (celcius)
[I2C] Sample 16: 39.000000 (celcius)
[I2C] Sample 17: 39.000000 (celcius)
[I2C] Sample 18: 39.000000 (celcius)
[I2C] Sample 19: 39.000000 (celcius)
All tests have passed!!

\endcode
