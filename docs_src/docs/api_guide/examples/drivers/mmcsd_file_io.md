# MMCSD FILE IO {#EXAMPLES_DRIVERS_MMCSD_FILE_IO}

[TOC]

# Introduction

This example demonstrates file I/O operations using the FreeRTOS+FAT file system to the SD Card inserted. The MMCSD instance and the media connected to it are selectable via Sysconfig. In the Sysconfig of the example, user just needs to add the FreeRTOS+FAT module. The MMCSD module instance will be added automatically.

The example initially checks for a FAT partition in the media. If it doesn't find one, a 128 MB partition is created (FAT16). Then a file is created and some known data is written to that. Then the file is closed and re-opened for read back. The file is then read back and compared with the known data.

When the comparison match, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_MMCSD_FILE_IO_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mmcsd/mmcsd_file_io

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mmcsd/mmcsd_file_io

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mmcsd/mmcsd_file_io

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mmcsd/mmcsd_file_io

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MMCSD_PAGE

# Sample Output

\code
[MMCSD FILE IO] Starting...
All tests have passed!!
\endcode