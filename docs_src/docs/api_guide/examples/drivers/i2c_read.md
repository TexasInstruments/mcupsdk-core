# I2C Read {#EXAMPLES_DRIVERS_I2C_READ}

[TOC]

# Introduction

This example demonstrates reading data from I2C based EEPROM devices present in the board.
Application reads 20 samples from the EEPROM and exits.

\cond SOC_AM64X
\attention For AM64xSK, Board ID EEPROM address is 0x51u.
\endcond

\cond SOC_AM62X
\attention For AM62x, Board ID EEPROM address is 0x51u.

SCL and SDA pins of MCU_I2C0 are available at the MCU_HEADER in the board.
MCU_I2C0 can be connected to Board ID EEPROM through making the following jumper connections as shown in the image.
 - Connect Pin 21 of J9 (MCU_I2C0_SDA) to Pin 27 of J3.
 - Connect Pin 24 of J9 (MCU_I2C0_SCL) to Pin 28 of J3.

  \imageStyle{i2c_read_am62x_sk.png,width:30%}
  \image html i2c_read_am62x_sk.png "JUMPER CONNECTIONS FOR I2C"

\endcond
# Supported Combinations {#EXAMPLES_DRIVERS_I2C_READ_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_read

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
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
[I2C] Sample 0: 255
[I2C] Sample 1: 255
[I2C] Sample 2: 255
[I2C] Sample 3: 255
[I2C] Sample 4: 255
[I2C] Sample 5: 255
[I2C] Sample 6: 255
[I2C] Sample 7: 255
[I2C] Sample 8: 255
[I2C] Sample 9: 255
[I2C] Sample 10: 255
[I2C] Sample 11: 255
[I2C] Sample 12: 255
[I2C] Sample 13: 255
[I2C] Sample 14: 255
[I2C] Sample 15: 255
[I2C] Sample 16: 255
[I2C] Sample 17: 255
[I2C] Sample 18: 255
[I2C] Sample 19: 255
[I2C] Read data ... DONE !!!All tests have passed!!
\endcode
