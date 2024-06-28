# FileX Hello World Example {#EXAMPLES_ECLIPSE_THREADX_FILEX_HELLO_WORLD}

[TOC]

# Introduction

This example demonstrates file I/O operations using the FileX file system. The test iterates over the FX_MEDIA instance table and executes the test on each available media. **IMPORTANT: the content of all avaiable media will be overwritten!**
Tested media can be added/removed via Sysconfig. By default all supported media on the board are tested. If an existing file system is found on the tested media, that file system is mounted. If no valid file system is found, the media is first formatted.

A test file is created (or truncated if it already exists). A buffer with some known data pattern is repeatedly written to the test file and then read back and compared to the original data.

# Supported Combinations {#EXAMPLES_ECLIPSE_THREADX_FILEX_HELLO_WORLD_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 threadx
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/eclipse_threadx/filex/hello_world

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

- \ref ECLIPSE_THREADX_FILEX
- \ref DRIVERS_MMCSD_PAGE

# Sample Output

\code
[FILEX HELLO WORLD] Hello world example started...
All tests have passed!!
\endcode