#  Hello World Project {#EXAMPLES_HELLO_WORLD}

[TOC]

# Introduction

This example just does driver and board initialization and prints the string, Hello World! on UART console.


# Supported Combinations {#EXAMPLES_HELLO_WORLD_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/hello_world/

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/hello_world/

\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 ^              | a53ss0-1 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/hello_world/

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/hello_world/

\endcond

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
\cond SOC_AM64X
**a53 core0:** on UART0(/dev/ttyUSB0)
\code
Hello World from a53_core0!
\endcode
**a53 core1:** on UART1(/dev/ttyUSB2)
\code
Hello World from a53_core1!
\endcode
\endcond
