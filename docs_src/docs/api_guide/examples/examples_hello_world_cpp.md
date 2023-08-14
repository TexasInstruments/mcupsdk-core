#  Hello World C++ Project {#EXAMPLES_HELLO_WORLD_CPP}

[TOC]

# Introduction

This example is compiled with the C++ compiler options.
It does driver and board initialization and prints the string, Hello World! on UART console.

# Supported Combinations {#EXAMPLES_HELLO_WORLD_CPP_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/hello_world_cpp

\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/hello_world_cpp

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/hello_world_cpp

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/hello_world_cpp

\endcond
\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/hello_world_cpp

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

# C++ Build Details

In this example the "hello_world.cpp" and "main.cpp" files are built with the c++ compiler flags.
The example will link with the prebuilt mcu sdk libs.

## Flags used for C++ build

Below are the flags used for the files to build with c++ flags along with the other flags used for c files.
-   -x c++
-   -Wno-c99-designator
-   -Wno-extern-c-compat
-   -Wno-c++11-narrowing
-   -Wno-reorder-init-list
-   -Wno-deprecated-register
-   -Wno-writable-strings
-   -Wno-enum-compare
-   -Wno-reserved-user-defined-literal
-   -Wno-unused-const-variable

## Building the mcu sdk libs with C++ flsgs.

The pre built libs provided in mcu sdk are built with the C flags.
However the application can re build the libs using the C++ flags.
The application needs to pass the below additional flag for the make commands used for building libs

\code
CPLUSPLUS_BUILD=yes
\endcode

note: Before re building the libs you need to clean all the pre built libs.

\attention All the MCU+ SDK libs are written in C. So if they're built with C++ flags, it is
possible that a linking error might happen during the application build because of the
name mangling done for C++ linkage. In this case, unless there is a requirement, it is recommended
to keep the libs built using C flags only to force the C linkage. For C++ linkage of libraries,
currently only the basic libs required for this example to build are tested.

## Building an application using c++ flags

Hello world cpp example is provided as referance to building application using c++ flags.
If you want to build the application using the c++ flags please refer the makefile
In the makefile the files with extn .cpp are compiled with the c++ flags and files with .c extn are compiled with c flags.

\code
examples\hello_world_cpp\<soc>\<core>_<os>\ti-arm-clang\makefile
\endcode
