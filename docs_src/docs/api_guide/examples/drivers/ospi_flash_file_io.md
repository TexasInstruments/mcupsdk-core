# OSPI Flash File IO {#EXAMPLES_DRIVERS_OSPI_FLASH_FILE_IO}

[TOC]

# Introduction

This example demonstrates basic read write file operation to the OSPI flash using LittleFS library. Wrapper file operation APIs for the littleFS driver files such as program, read, erase and sync are implemented using Flash APIs. The underlying OSPI reads and writes are taken care by the flash APIs. The block device configuration for LittleFS library is provided from the example itself.

The example writes a counter called fileWriteCounter into a file called 'test_file_write_count'. This counter is initialized to 0 and incremented for APP_OSPI_FILE_WRITE_COUNT number of iterations. This value is read to check if the counter is actually incremented.

When fileWriteCounter matches the given APP_OSPI_FILE_WRITE_COUNT macro value, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_FLASH_FILE_IO_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_file_io

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_file_io

 \endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_file_io

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_OSPI_PAGE

# Sample Output

\code
APP_OSPI_FILE_WRITE_COUNT: 32
fileWriteCounter: 1
fileWriteCounter: 2
fileWriteCounter: 3
fileWriteCounter: 4
fileWriteCounter: 5
fileWriteCounter: 6
fileWriteCounter: 7
fileWriteCounter: 8
fileWriteCounter: 9
fileWriteCounter: 10
fileWriteCounter: 11
fileWriteCounter: 12
fileWriteCounter: 13
fileWriteCounter: 14
fileWriteCounter: 15
fileWriteCounter: 16
fileWriteCounter: 17
fileWriteCounter: 18
fileWriteCounter: 19
fileWriteCounter: 20
fileWriteCounter: 21
fileWriteCounter: 22
fileWriteCounter: 23
fileWriteCounter: 24
fileWriteCounter: 25
fileWriteCounter: 26
fileWriteCounter: 27
fileWriteCounter: 28
fileWriteCounter: 29
fileWriteCounter: 30
fileWriteCounter: 31
fileWriteCounter: 32
fileWriteCounter reaches APP_OSPI_FILE_WRITE_COUNT
All tests have passed!!
\endcode