#  Hello World Board Sysconfig Project {#EXAMPLES_HELLO_WORLD_BOARD_SYSCFG}

[TOC]

# Introduction

This example just does driver and board initialization and prints the string, Hello World! on UART console. The UART module uses the on board XDS110 hardware which is selected via the board view feature in Sysconfig.

\image html use_hw_example.png

Refer to \ref EVM_SYSCONFIG_GUIDE for additional information.

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/hello_world_board_syscfg/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Hello World!
\endcode
