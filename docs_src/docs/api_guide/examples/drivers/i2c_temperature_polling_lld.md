# I2C Temperature Polling LLD {#EXAMPLES_DRIVERS_I2C_TEMPERATURE_POLLING_LLD}

[TOC]

# Introduction

This example demonstrates probing of sensor via I2C and reading data from it.
It makes use of the LLD driver and it's polling transfer API.
The application reads data from the temperature sensor in the EVM.
Application reads 20 samples from the sensor and exits.

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_TEMPERATURE_POLLING_LLD_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature_polling_lld

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature_polling_lld

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature_polling_lld

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_temperature_polling_lld

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
[I2C] I2C Temperature Read test Started in polling mode ... !!!
[I2C] Temperature sensor found at device address 0x4c
[I2C] Sample 0: 44.000000 (celcius)
[I2C] Sample 1: 44.000000 (celcius)
[I2C] Sample 2: 44.000000 (celcius)
[I2C] Sample 3: 44.000000 (celcius)
[I2C] Sample 4: 44.000000 (celcius)
[I2C] Sample 5: 44.000000 (celcius)
[I2C] Sample 6: 44.000000 (celcius)
[I2C] Sample 7: 44.000000 (celcius)
[I2C] Sample 8: 44.000000 (celcius)
[I2C] Sample 9: 44.000000 (celcius)
[I2C] Sample 10: 44.000000 (celcius)
[I2C] Sample 11: 44.000000 (celcius)
[I2C] Sample 12: 44.000000 (celcius)
[I2C] Sample 13: 44.000000 (celcius)
[I2C] Sample 14: 44.000000 (celcius)
[I2C] Sample 15: 44.000000 (celcius)
[I2C] Sample 16: 44.000000 (celcius)
[I2C] Sample 17: 44.000000 (celcius)
[I2C] Sample 18: 44.000000 (celcius)
[I2C] Sample 19: 44.000000 (celcius)
All tests have passed!!
\endcode

\endcond

\cond SOC_AM273X || SOC_AWR294X

\code
[I2C] I2C Temperature Read test Started in polling mode ... !!!
[I2C] Temperature sensor found at device address 0x49
[I2C] Sample 0: 44.000000 (celcius)
[I2C] Sample 1: 44.000000 (celcius)
[I2C] Sample 2: 44.000000 (celcius)
[I2C] Sample 3: 44.000000 (celcius)
[I2C] Sample 4: 44.000000 (celcius)
[I2C] Sample 5: 44.000000 (celcius)
[I2C] Sample 6: 44.000000 (celcius)
[I2C] Sample 7: 44.000000 (celcius)
[I2C] Sample 8: 44.000000 (celcius)
[I2C] Sample 9: 44.000000 (celcius)
[I2C] Sample 10: 44.000000 (celcius)
[I2C] Sample 11: 44.000000 (celcius)
[I2C] Sample 12: 44.000000 (celcius)
[I2C] Sample 13: 44.000000 (celcius)
[I2C] Sample 14: 44.000000 (celcius)
[I2C] Sample 15: 44.000000 (celcius)
[I2C] Sample 16: 44.000000 (celcius)
[I2C] Sample 17: 44.000000 (celcius)
[I2C] Sample 18: 44.000000 (celcius)
[I2C] Sample 19: 44.000000 (celcius)
All tests have passed!!
\endcode

\endcond